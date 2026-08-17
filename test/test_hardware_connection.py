"""Tests for the two-stage AGX hardware connection."""

import pytest

from armbycontroller.hardware.connection import connect_arm_two_stage
from armbycontroller.hardware.connection import FirmwareDetectionError
from armbycontroller.hardware.connection import firmware_profile_from_info


@pytest.mark.parametrize(
    ("robot_model", "software_version", "expected"),
    [
        ("nero", "1.10", "default"),
        ("nero", "1.11", "v111"),
        ("nero", "1.12", "v112"),
        ("nero", "1.121", "v112"),
        ("nero", "1.20", "v120"),
        ("piper_l", "S-V1.8-2", "default"),
        ("piper_l", "S-V1.8-3", "v183"),
        ("piper_l", "S-V1.8-8", "v188"),
        ("piper_l", "S-V1.8-9", "v189"),
        ("piper_l", "S-V1.9-0", "v189"),
    ],
)
def test_firmware_profile_matches_sdk_ranges(
    robot_model, software_version, expected
):
    assert firmware_profile_from_info(
        robot_model, {"software_version": software_version}
    ) == expected


@pytest.mark.parametrize(
    ("robot_model", "firmware_info", "expected_profile"),
    [
        (
            "nero",
            {"software_version": "1.11", "serial_number": "nero-01"},
            "v111",
        ),
        (
            "piper_l",
            {
                "software_version": "S-V1.8-8",
                "hardware_version": "H-V1.2-1",
            },
            "v188",
        ),
    ],
)
def test_two_stage_connection_saves_probe_data_and_reconnects_with_it(
    robot_model, firmware_info, expected_profile
):
    events = []
    reports = []
    created = []
    configs = []

    class FakeArm:
        def __init__(self, name):
            self.name = name
            self.connected = False

        def connect(self):
            events.append(f"connect:{self.name}")
            self.connected = True

        def disconnect(self):
            events.append(f"disconnect:{self.name}")
            self.connected = False

        def enable(self):
            events.append(f"enable:{self.name}")
            return False

        def disable(self):
            events.append(f"disable:{self.name}")
            return False

        def get_firmware(self, timeout, min_interval):
            assert timeout > 0.0
            assert min_interval == 0.0
            events.append(f"firmware:{self.name}")
            return firmware_info

    class FakeFactory:
        @staticmethod
        def create_arm(config):
            arm = FakeArm(f"arm{len(created) + 1}")
            created.append(arm)
            events.append(
                f"create:{arm.name}:{config['firmeware_version']}"
            )
            return arm

    def fake_config_factory(**values):
        configs.append(dict(values))
        return dict(values)

    result = connect_arm_two_stage(
        robot_model=robot_model,
        arm_model=f"sdk-{robot_model}",
        firmware_profiles={
            "default": "default",
            "v111": "v111",
            "v112": "v112",
            "v120": "v120",
            "v183": "v183",
            "v188": "v188",
            "v189": "v189",
        },
        can_interface="can-test",
        arm_factory=FakeFactory,
        config_factory=fake_config_factory,
        report=reports.append,
        sleep=lambda duration: events.append(f"sleep:{duration}"),
    )

    assert len(created) == 2
    assert created[0] is not created[1]
    expected_events = [
        "create:arm1:default",
        "connect:arm1",
        "enable:arm1",
        "firmware:arm1",
        "disable:arm1",
    ]
    expected_events.extend([
        "disconnect:arm1",
        "sleep:0.5",
        f"create:arm2:{expected_profile}",
        "connect:arm2",
    ])
    assert events == expected_events
    assert not created[0].connected
    assert created[1].connected
    assert result.arm is created[1]
    assert result.firmware_info == firmware_info
    assert result.firmware_info is not firmware_info
    assert result.firmware_profile == expected_profile
    assert configs[0]["firmeware_version"] == "default"
    assert configs[1]["firmeware_version"] == expected_profile
    expected_reports = [
        "firmware probe connection: "
        f"{robot_model} on can-test with default profile",
        f"{robot_model} firmware probe: sending one temporary enable request",
        f"{robot_model} firmware probe enable request result: False",
        "firmware probe query 1: requesting device data "
        "with timeout=1.000 s",
        f"firmware probe query 1 received: {firmware_info!r}",
        f"firmware probe data saved: {firmware_info}; "
        f"selected profile={expected_profile}",
        f"{robot_model} firmware probe disable request result: False",
    ]
    expected_reports.extend([
        "firmware probe disconnected",
        "waiting 0.500 s before formal connection",
        "formal control connection: "
        f"{robot_model} on can-test with detected profile {expected_profile}",
    ])
    assert reports == expected_reports


def test_probe_reports_each_missing_firmware_response_before_timeout():
    reports = []
    current_time = [0.0]

    class FakeArm:
        def connect(self):
            pass

        def disconnect(self):
            pass

        def enable(self):
            return False

        def disable(self):
            return False

        def get_firmware(self, timeout, min_interval):
            del timeout, min_interval
            return None

    class FakeFactory:
        @staticmethod
        def create_arm(config):
            del config
            return FakeArm()

    def monotonic():
        return current_time[0]

    def sleep(duration):
        current_time[0] += duration

    with pytest.raises(
        FirmwareDetectionError,
        match=r"timed out after 0\.250 s and 3 queries",
    ):
        connect_arm_two_stage(
            robot_model="nero",
            arm_model="sdk-nero",
            firmware_profiles={"default": "default"},
            can_interface="can-test",
            probe_timeout=0.25,
            probe_poll_period=0.1,
            arm_factory=FakeFactory,
            config_factory=lambda **values: values,
            report=reports.append,
            monotonic=monotonic,
            sleep=sleep,
        )

    assert [message for message in reports if "no response" in message] == [
        "firmware probe query 1: no response",
        "firmware probe query 2: no response",
        "firmware probe query 3: no response",
    ]
    assert reports[-1] == "firmware probe disconnected"


def test_reconnect_delay_is_configurable_and_precedes_formal_creation():
    events = []

    class FakeArm:
        def __init__(self, name):
            self.name = name

        def connect(self):
            events.append(f"connect:{self.name}")

        def disconnect(self):
            events.append(f"disconnect:{self.name}")

        def enable(self):
            events.append(f"enable:{self.name}")
            return False

        def disable(self):
            events.append(f"disable:{self.name}")
            return False

        def get_firmware(self, timeout, min_interval):
            del timeout, min_interval
            return {"software_version": "1.11"}

    class FakeFactory:
        created = 0

        @classmethod
        def create_arm(cls, config):
            del config
            cls.created += 1
            events.append(f"create:arm{cls.created}")
            return FakeArm(f"arm{cls.created}")

    connect_arm_two_stage(
        robot_model="nero",
        arm_model="sdk-nero",
        firmware_profiles={"default": "default", "v111": "v111"},
        can_interface="can-test",
        reconnect_delay=0.5,
        arm_factory=FakeFactory,
        config_factory=lambda **values: values,
        sleep=lambda duration: events.append(f"sleep:{duration}"),
    )

    assert events == [
        "create:arm1",
        "connect:arm1",
        "enable:arm1",
        "disable:arm1",
        "disconnect:arm1",
        "sleep:0.5",
        "create:arm2",
        "connect:arm2",
    ]


@pytest.mark.parametrize("delay", [-0.1, float("nan"), float("inf")])
def test_reconnect_delay_rejects_invalid_values(delay):
    with pytest.raises(ValueError, match="reconnect_delay"):
        connect_arm_two_stage(
            robot_model="nero",
            arm_model="sdk-nero",
            firmware_profiles={"default": "default"},
            can_interface="can-test",
            reconnect_delay=delay,
            arm_factory=object(),
            config_factory=lambda **values: values,
        )


def test_probe_is_disconnected_when_firmware_data_is_invalid():
    events = []

    class FakeArm:
        def connect(self):
            events.append("connect")

        def disconnect(self):
            events.append("disconnect")

        def enable(self):
            events.append("enable")
            return False

        def disable(self):
            events.append("disable")
            return False

        def get_firmware(self, timeout, min_interval):
            del timeout, min_interval
            return {"hardware_version": "unknown"}

    class FakeFactory:
        @staticmethod
        def create_arm(config):
            del config
            return FakeArm()

    with pytest.raises(FirmwareDetectionError, match="software_version"):
        connect_arm_two_stage(
            robot_model="nero",
            arm_model="sdk-nero",
            firmware_profiles={"default": "default"},
            can_interface="can-test",
            arm_factory=FakeFactory,
            config_factory=lambda **values: values,
        )

    assert events == ["connect", "enable", "disable", "disconnect"]
