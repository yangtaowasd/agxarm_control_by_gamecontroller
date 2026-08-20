#!/usr/bin/env python3
"""Start the unified Nero/Piper-L ROS 2 controller node."""

import rclpy
from rclpy.executors import ExternalShutdownException

from armbycontroller.ros.node import ArmKeyboardController


def main(args=None):
    """Run the controller until ROS or the operator requests shutdown."""
    rclpy.init(args=args)
    node = ArmKeyboardController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
