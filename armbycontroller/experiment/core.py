"""Transport-independent experiment lifecycle and metrics."""

from collections import Counter
from datetime import datetime
from datetime import timezone
import json
import math
from pathlib import Path
import re
from typing import Mapping
from typing import Protocol
from typing import runtime_checkable
from uuid import uuid4

import numpy as np


def _json_object(value, name):
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    try:
        encoded = json.dumps(value, allow_nan=False, sort_keys=True)
    except (TypeError, ValueError) as error:
        raise ValueError(
            f"{name} must be finite and JSON-compatible"
        ) from error
    decoded = json.loads(encoded)
    if not isinstance(decoded, dict):
        raise ValueError(f"{name} must encode as a JSON object")
    return decoded


def _atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


@runtime_checkable
class ExperimentSink(Protocol):
    """Persistence seam for a complete experiment run."""

    def open(self, manifest):
        """Open a run and persist its immutable manifest."""

    def write_sample(self, sample):
        """Persist one ordered control sample."""

    def write_event(self, event):
        """Persist one ordered non-periodic event."""

    def close(self, summary):
        """Finish the run and persist its summary."""


class MemoryExperimentSink:
    """In-memory adapter used by tests and short programmatic experiments."""

    def __init__(self):
        self.manifest = None
        self.samples = []
        self.events = []
        self.summary = None

    def open(self, manifest):
        if self.manifest is not None:
            raise RuntimeError("memory experiment sink is already open")
        self.manifest = _json_object(manifest, "manifest")

    def write_sample(self, sample):
        self.samples.append(_json_object(sample, "sample"))

    def write_event(self, event):
        self.events.append(_json_object(event, "event"))

    def close(self, summary):
        self.summary = _json_object(summary, "summary")


class JsonlExperimentSink:
    """Filesystem adapter with JSON manifest/summary and streaming JSONL."""

    def __init__(self, output_root, run_id, flush_every=1):
        self.output_root = Path(output_root).expanduser().resolve()
        self.run_id = str(run_id)
        self.flush_every = int(flush_every)
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", self.run_id):
            raise ValueError("run_id may contain only letters, digits, _.-")
        if self.flush_every < 1:
            raise ValueError("flush_every must be at least one")
        self.run_directory = self.output_root / self.run_id
        self._samples = None
        self._events = None
        self._pending = 0

    def open(self, manifest):
        if self._samples is not None:
            raise RuntimeError("JSONL experiment sink is already open")
        self.run_directory.mkdir(parents=True, exist_ok=False)
        _atomic_json(
            self.run_directory / "manifest.json",
            _json_object(manifest, "manifest"),
        )
        self._samples = (self.run_directory / "samples.jsonl").open(
            "x", encoding="utf-8"
        )
        self._events = (self.run_directory / "events.jsonl").open(
            "x", encoding="utf-8"
        )

    def _write(self, stream, value, name):
        if stream is None:
            raise RuntimeError("JSONL experiment sink is not open")
        stream.write(
            json.dumps(
                _json_object(value, name),
                allow_nan=False,
                ensure_ascii=False,
                separators=(",", ":"),
                sort_keys=True,
            )
            + "\n"
        )
        self._pending += 1
        if self._pending >= self.flush_every:
            self._samples.flush()
            self._events.flush()
            self._pending = 0

    def write_sample(self, sample):
        self._write(self._samples, sample, "sample")

    def write_event(self, event):
        self._write(self._events, event, "event")

    def close(self, summary):
        if self._samples is None or self._events is None:
            raise RuntimeError("JSONL experiment sink is not open")
        self._samples.flush()
        self._events.flush()
        self._samples.close()
        self._events.close()
        self._samples = None
        self._events = None
        _atomic_json(
            self.run_directory / "summary.json",
            _json_object(summary, "summary"),
        )


class ExperimentRun:
    """Own one manifest, ordered evidence stream, events, and summary."""

    schema_version = 1

    def __init__(self, name, sink, metadata=None, *, run_id=None, clock=None):
        name = str(name).strip()
        if not name:
            raise ValueError("experiment name must not be empty")
        if not isinstance(sink, ExperimentSink):
            raise TypeError("sink must satisfy ExperimentSink")
        self.name = name
        self.sink = sink
        self.metadata = _json_object(metadata or {}, "metadata")
        self.run_id = str(run_id or self.new_run_id(name))
        self.clock = clock or (lambda: datetime.now(timezone.utc).timestamp())
        self.started_at = None
        self.closed = False
        self.sample_count = 0
        self.event_count = 0
        self.controllers = Counter()
        self.command_modes = Counter()
        self.period_sum = 0.0
        self.period_count = 0
        self.period_min = math.inf
        self.period_max = 0.0
        self.squared_position_error_sum = 0.0
        self.position_error_values = 0
        self.maximum_absolute_position_error = 0.0
        self.maximum_absolute_estimated_torque = 0.0

    @staticmethod
    def new_run_id(name):
        """Return a filesystem-safe, collision-resistant UTC run identifier."""
        safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "-", name).strip("-.")
        safe_name = safe_name or "experiment"
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        return f"{stamp}-{safe_name}-{uuid4().hex[:8]}"

    @property
    def active(self):
        return self.started_at is not None and not self.closed

    def start(self):
        if self.started_at is not None:
            raise RuntimeError("experiment run has already started")
        started_at = float(self.clock())
        if not math.isfinite(started_at):
            raise ValueError("experiment clock returned a non-finite value")
        manifest = {
            "schema_version": self.schema_version,
            "run_id": self.run_id,
            "experiment_name": self.name,
            "started_at": started_at,
            "metadata": self.metadata,
        }
        self.sink.open(manifest)
        self.started_at = started_at
        return manifest

    def _require_active(self):
        if not self.active:
            raise RuntimeError("experiment run is not active")

    def record_sample(self, sample):
        self._require_active()
        value = _json_object(sample, "sample")
        value["sequence"] = self.sample_count
        self.sink.write_sample(value)
        self.sample_count += 1
        self.controllers[str(value.get("controller", "unknown"))] += 1
        command = value.get("command", {})
        if isinstance(command, dict):
            self.command_modes[str(command.get("mode", "unknown"))] += 1
        period = float(value.get("period", 0.0))
        if math.isfinite(period) and period > 0.0:
            self.period_sum += period
            self.period_count += 1
            self.period_min = min(self.period_min, period)
            self.period_max = max(self.period_max, period)
        self._update_error_metrics(value)

    def _update_error_metrics(self, sample):
        try:
            state = sample["state"]
            if not bool(state.get("position_valid", True)):
                return
            measured = np.asarray(state["position"], dtype=float)
            target = np.asarray(sample["reference"]["position"], dtype=float)
            if measured.shape != target.shape or measured.ndim != 1:
                return
            error = target - measured
            if not np.all(np.isfinite(error)):
                return
            self.squared_position_error_sum += float(error @ error)
            self.position_error_values += error.size
            self.maximum_absolute_position_error = max(
                self.maximum_absolute_position_error,
                float(np.max(np.abs(error), initial=0.0)),
            )
            estimated = sample.get("command", {}).get("estimated_torque")
            if estimated is not None:
                torque = np.asarray(estimated, dtype=float)
                if torque.ndim == 1 and np.all(np.isfinite(torque)):
                    self.maximum_absolute_estimated_torque = max(
                        self.maximum_absolute_estimated_torque,
                        float(np.max(np.abs(torque), initial=0.0)),
                    )
        except (KeyError, TypeError, ValueError):
            return

    def record_event(self, event, **fields):
        self._require_active()
        value = {
            "sequence": self.event_count,
            "timestamp": float(self.clock()),
            "event": str(event),
            **fields,
        }
        self.sink.write_event(_json_object(value, "event"))
        self.event_count += 1

    def close(self, outcome="completed", **fields):
        self._require_active()
        finished_at = float(self.clock())
        mean_period = (
            self.period_sum / self.period_count if self.period_count else None
        )
        position_rmse = (
            math.sqrt(
                self.squared_position_error_sum / self.position_error_values
            )
            if self.position_error_values
            else None
        )
        summary = {
            "schema_version": self.schema_version,
            "run_id": self.run_id,
            "experiment_name": self.name,
            "outcome": str(outcome),
            "started_at": self.started_at,
            "finished_at": finished_at,
            "duration": finished_at - self.started_at,
            "sample_count": self.sample_count,
            "event_count": self.event_count,
            "controller_counts": dict(self.controllers),
            "command_mode_counts": dict(self.command_modes),
            "period": {
                "minimum": self.period_min if self.period_count else None,
                "mean": mean_period,
                "maximum": self.period_max if self.period_count else None,
            },
            "metrics": {
                "joint_position_rmse_rad": position_rmse,
                "maximum_absolute_joint_position_error_rad": (
                    self.maximum_absolute_position_error
                ),
                "maximum_absolute_estimated_torque_nm": (
                    self.maximum_absolute_estimated_torque
                ),
            },
            **fields,
        }
        self.sink.close(_json_object(summary, "summary"))
        self.closed = True
        return summary
