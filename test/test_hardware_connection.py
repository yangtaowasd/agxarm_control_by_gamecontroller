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
        sleep=lambda duration: events.append(f"sleep:{duration}"),
    )

    assert len(created) == 2
    assert created[0] is not created[1]
    assert events == [
        "create:arm1:default",
        "connect:arm1",
        "firmware:arm1",
        "disconnect:arm1",
        "sleep:0.5",
        f"create:arm2:{expected_profile}",
        "connect:arm2",
    ]
    assert not created[0].connected
    assert created[1].connected
    assert result.arm is created[1]
    assert result.firmware_info == firmware_info
    assert result.firmware_info is not firmware_info
    assert result.firmware_profile == expected_profile
    assert configs[0]["firmeware_version"] == "default"
    assert configs[1]["firmeware_version"] == expected_profile


def test_reconnect_delay_is_configurable_and_precedes_formal_creation():
    events = []

    class FakeArm:
        def __init__(self, name):
            self.name = name

        def connect(self):
            events.append(f"connect:{self.name}")

        def disconnect(self):
            events.append(f"disconnect:{self.name}")

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

    assert events == ["connect", "disconnect"]
