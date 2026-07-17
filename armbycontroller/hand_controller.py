#!/usr/bin/env python3
"""Control a BrainCo Revo2 Touch hand through an AGX arm bridge."""

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


MOTOR_NAMES = (
    "thumb",
    "thumb_aux",
    "index",
    "middle",
    "ring",
    "pinky",
)
TOUCH_NAMES = ("thumb", "index", "middle", "ring", "pinky")


def resolve_normalized_unit_mode(hand):
    """Resolve the BrainCo enum across pyAgxArm grouped-import fallbacks."""
    mode_type = getattr(hand, "FingerUnitMode", None)
    normalized = getattr(mode_type, "Normalized", None)
    if normalized is not None:
        return normalized

    # pyAgxArm 1.0.0 imports several optional BrainCo types as one group.
    # bc-stark-sdk 1.4.5 lacks HandType, making that grouped import fall back
    # to object even though FingerUnitMode itself is available.
    from bc_stark_sdk.main_mod import FingerUnitMode

    return FingerUnitMode.Normalized


class Revo2HandTest(Node):
    """Control and monitor Revo2 Touch through pyAgxArm's Nero bridge."""

    def __init__(self):
        super().__init__("hand_controller")

        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("firmware", "v111")
        self.declare_parameter("hand_side", "auto")
        self.declare_parameter("hardware_type", "revo2_touch")
        self.declare_parameter("execute_motion", False)
        self.declare_parameter("touch_period", 0.05)
        self.declare_parameter("state_period", 0.5)
        self.declare_parameter("auto_cycle", False)
        self.declare_parameter("motion_period", 3.0)
        self.declare_parameter("open_positions", [400, 400, 0, 0, 0, 0])
        self.declare_parameter(
            "fist_positions", [500, 500, 1000, 1000, 1000, 1000]
        )

        self.can_interface = str(self.get_parameter("can_interface").value)
        self.firmware = str(self.get_parameter("firmware").value).lower()
        self.hand_side = str(self.get_parameter("hand_side").value).lower()
        self.hardware_type = str(
            self.get_parameter("hardware_type").value
        ).lower()
        self.execute_motion = bool(self.get_parameter("execute_motion").value)
        self.touch_period = max(
            0.01, float(self.get_parameter("touch_period").value)
        )
        self.state_period = max(
            0.1, float(self.get_parameter("state_period").value)
        )
        self.auto_cycle = bool(self.get_parameter("auto_cycle").value)
        self.motion_period = max(
            0.5, float(self.get_parameter("motion_period").value)
        )
        self.open_positions = self._read_positions("open_positions")
        self.fist_positions = self._read_positions("fist_positions")

        if self.hand_side not in ("auto", "left", "right"):
            raise ValueError("hand_side must be auto, left, or right")
        if self.hardware_type not in ("auto", "revo2_touch"):
            raise ValueError("hardware_type must be auto or revo2_touch")

        self.touch_pub = self.create_publisher(String, "/revo2/touch", 10)
        self.motor_pub = self.create_publisher(
            String, "/revo2/motor_status", 10
        )
        self.create_subscription(
            String, "/revo2/command", self.command_callback, 10
        )

        self.robot = None
        self.hand = None
        self.next_cycle_is_fist = True
        try:
            self.connect()
        except Exception:
            self.close()
            raise

        self.create_timer(self.touch_period, self.publish_touch)
        self.create_timer(self.state_period, self.publish_motor_status)
        if self.auto_cycle:
            self.create_timer(self.motion_period, self.cycle_motion)

        if not self.execute_motion:
            self.get_logger().warning(
                "execute_motion=false: open/fist commands are dry-run only"
            )

    def _read_positions(self, name):
        values = [int(value) for value in self.get_parameter(name).value]
        if (
            len(values) != 6
            or any(value < 0 or value > 1000 for value in values)
        ):
            raise ValueError(f"{name} must contain six values in [0, 1000]")
        return values

    def connect(self):
        """Connect Nero first, then create the Revo2 Touch bridge."""
        from pyAgxArm import AgxArmFactory, ArmModel, NeroFW
        from pyAgxArm import create_agx_arm_config

        firmware_map = {
            "default": NeroFW.DEFAULT,
            "v111": NeroFW.V111,
            "v112": NeroFW.V112,
        }
        if self.firmware not in firmware_map:
            raise ValueError("firmware must be default, v111, or v112")

        config = create_agx_arm_config(
            robot=ArmModel.NERO,
            firmeware_version=firmware_map[self.firmware],
            interface="socketcan",
            channel=self.can_interface,
        )
        self.robot = AgxArmFactory.create_arm(config)
        self.robot.connect()
        time.sleep(0.5)

        self.hand = self.robot.init_effector(
            self.robot.OPTIONS.EFFECTOR.REVO2_TOUCH
        )
        self.hand.set_hand_side(
            None if self.hand_side == "auto" else self.hand_side
        )

        if self.hardware_type == "revo2_touch":
            from bc_stark_sdk.main_mod import StarkHardwareType

            # Some bridged hands report an empty SN. BrainCo then defaults to
            # Revo2Basic, so explicitly select the capacitive Touch protocol.
            self.hand.run_sdk(
                self.hand.client.set_hardware_type,
                StarkHardwareType.Revo2Touch,
            )

        # Calling get_device_info once also caches the hardware capability used
        # by the official BrainCo SDK behind the Nero bridge.
        device_info = self.hand.run_sdk(self.hand.client.get_device_info)
        if device_info is None:
            self.close()
            raise RuntimeError(
                "Revo2 Touch was not found through the Nero bridge"
            )

        self.hand.run_sdk(
            self.hand.client.set_finger_unit_mode,
            resolve_normalized_unit_mode(self.hand),
        )
        self.hand.run_sdk(self.hand.client.touch_sensor_setup, 0x1F)
        time.sleep(1.0)
        enabled = self.hand.run_sdk(self.hand.client.get_touch_sensor_enabled)
        if enabled is None or enabled & 0x1F != 0x1F:
            self.close()
            raise RuntimeError(
                f"failed to enable five touch sensors: {enabled}"
            )

        self.get_logger().info(
            f"Nero/Revo2 connected: can={self.can_interface}, "
            f"firmware={self.firmware}, side={self.hand.hand_side}, "
            f"slave_id={self.hand.slave_id}, "
            f"hw_override={self.hardware_type}, "
            f"device={device_info.description}"
        )

    def command_callback(self, msg):
        """Handle the textual commands open and fist."""
        command = msg.data.strip().lower()
        if command == "open":
            self.send_positions("open", self.open_positions)
        elif command in ("fist", "close"):
            self.send_positions("fist", self.fist_positions)
        else:
            self.get_logger().warning(
                f"unknown command '{msg.data}'; expected 'open' or 'fist'"
            )

    def send_positions(self, name, positions):
        """Send six normalized positions through run_sdk/client."""
        if not self.execute_motion:
            self.get_logger().info(f"dry-run {name}: {positions}")
            return

        self.hand.run_sdk(
            self.hand.client.set_finger_positions,
            list(positions),
        )
        feedback = self.hand.run_sdk(self.hand.client.get_finger_positions)
        if feedback is None:
            self.get_logger().error(f"{name} command feedback failed")
            return
        self.get_logger().info(
            f"command sent: {name} {positions}, feedback={list(feedback)}"
        )

    def cycle_motion(self):
        """Alternate fist and open when auto_cycle is enabled."""
        if self.next_cycle_is_fist:
            self.send_positions("fist", self.fist_positions)
        else:
            self.send_positions("open", self.open_positions)
        self.next_cycle_is_fist = not self.next_cycle_is_fist

    def publish_touch(self):
        """Read and publish the five fingers' tactile data."""
        touch_items = self.hand.run_sdk(
            self.hand.client.get_touch_sensor_status
        )
        if touch_items is None:
            self.get_logger().warning(
                "touch read failed",
                throttle_duration_sec=1.0,
            )
            return
        if len(touch_items) != 5:
            self.get_logger().warning(
                f"expected five touch records, got {len(touch_items)}"
            )
            return

        fingers = {}
        for name, item in zip(TOUCH_NAMES, touch_items):
            fingers[name] = {
                "normal_force": [
                    int(item.normal_force1),
                    int(item.normal_force2),
                    int(item.normal_force3),
                ],
                "tangential_force": [
                    int(item.tangential_force1),
                    int(item.tangential_force2),
                    int(item.tangential_force3),
                ],
                "tangential_direction": [
                    int(item.tangential_direction1),
                    int(item.tangential_direction2),
                    int(item.tangential_direction3),
                ],
                "proximity": [
                    int(item.self_proximity1),
                    int(item.self_proximity2),
                    int(item.mutual_proximity),
                ],
                "status": int(item.status),
            }
        self._publish_json(
            self.touch_pub,
            {"timestamp": time.time(), "fingers": fingers},
        )

    def publish_motor_status(self):
        """Read all motor state fields every 0.5 seconds via run_sdk."""
        status = self.hand.run_sdk(self.hand.client.get_motor_status)
        if status is None:
            self.get_logger().warning(
                "motor status read failed",
                throttle_duration_sec=1.0,
            )
            return

        payload = {
            "timestamp": time.time(),
            "motor_names": MOTOR_NAMES,
            "positions": list(status.positions),
            "speeds": list(status.speeds),
            "currents": list(status.currents),
            "states": [
                {"value": int(state), "name": str(state)}
                for state in status.states
            ],
        }
        self._publish_json(self.motor_pub, payload)
        self.get_logger().info(json.dumps(payload, ensure_ascii=False))

    @staticmethod
    def _publish_json(publisher, payload):
        msg = String()
        msg.data = json.dumps(
            payload, ensure_ascii=False, separators=(",", ":")
        )
        publisher.publish(msg)

    def close(self):
        """Disconnect Nero; the Revo2 bridge shares this transport."""
        if self.robot is not None:
            self.robot.disconnect()
        self.hand = None
        self.robot = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Revo2HandTest()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
