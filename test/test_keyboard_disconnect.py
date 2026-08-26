"""Integration regression tests for the evdev keyboard reader."""

import os
from pathlib import Path
import signal
import struct
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import PackageNotFoundError
import pytest
import rclpy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Int32MultiArray


PACKAGE_NAME = "agxarm_control_by_gamecontroller"
KEY_W = 17
KEY_W_STATE_INDEX = 12


def _spin_until(node, predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return True
    return False


def test_evdev_disconnect_releases_pressed_keys_and_latches_zero(
    tmp_path, monkeypatch
):
    try:
        package_prefix = get_package_prefix(PACKAGE_NAME)
    except PackageNotFoundError:
        pytest.skip("keyboard executable is available after a package build")
    executable = (
        Path(package_prefix)
        / "lib"
        / PACKAGE_NAME
        / "keyboard"
    )
    if not executable.is_file():
        pytest.skip("keyboard executable is available after a package build")

    fifo = tmp_path / "keyboard-event-fifo"
    os.mkfifo(fifo)
    writer = os.open(fifo, os.O_RDWR | os.O_NONBLOCK)
    log_directory = tmp_path / "ros-log"
    log_directory.mkdir()
    monkeypatch.setenv("ROS_LOG_DIR", str(log_directory))
    monkeypatch.setenv("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))
    topic = f"keyboard_disconnect_{os.getpid()}"
    process = subprocess.Popen(
        [
            str(executable),
            "--ros-args",
            "-p",
            f"device:={fifo}",
            "-p",
            f"keyboard_topic:={topic}",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    messages = []
    rclpy.init()
    node = rclpy.create_node("keyboard_disconnect_test")
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )
    subscription = node.create_subscription(
        Int32MultiArray,
        topic,
        lambda message: messages.append(list(message.data)),
        qos,
    )
    del subscription

    try:
        event = struct.pack("llHHi", 0, 0, 1, KEY_W, 1)
        os.write(writer, event)
        assert _spin_until(
            node,
            lambda: any(
                state[KEY_W_STATE_INDEX] == 1 for state in messages
            ),
        )

        messages.clear()
        os.close(writer)
        writer = -1
        assert _spin_until(
            node,
            lambda: sum(
                state[KEY_W_STATE_INDEX] == 0 for state in messages
            ) >= 3,
        )
        assert all(state[KEY_W_STATE_INDEX] == 0 for state in messages[-3:])
    finally:
        if writer >= 0:
            os.close(writer)
        node.destroy_node()
        rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.communicate(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate(timeout=2.0)
