"""
PROS Arm Commercial License
Copyright 2023 帕亞科技 (PAIA-Tech) . All rights reserved.
Primary Author: 陳麒麟(Kylin Chen)
Contributor(s): 蘇文鈺(Alvin Su), 鍾博丞(Otis Chung)
This software and associated documentation files are proprietary to 帕亞科技 (PAIA-Tech),
and are not to be copied, reproduced, or disclosed to any third party, in whole or in part, without
express prior written permission. Use and modification of this software is permitted for licensed
users only, and must always remain proprietary to 帕亞科技 (PAIA-Tech).

This code is a node to read data from specific arm.
It will publish arm state.
"""

import math
import orjson
import rclpy
from rclpy.node import Node
from serial import Serial
from trajectory_msgs.msg import JointTrajectoryPoint
from .env import ARM_SERIAL_PORT_LEFT, ARM_SERIAL_PORT_RIGHT


class ArmSerialWriter(Node):
    def __init__(self):
        super().__init__('arm_serial_writer')

        # Set up the serial connection
        serial_port_left = self.declare_parameter('serial_port_left', ARM_SERIAL_PORT_LEFT).value
        serial_port_right = self.declare_parameter('serial_port_right', ARM_SERIAL_PORT_RIGHT).value
        self._serial_left = Serial(serial_port_left, 115200, timeout=0)
        self._serial_right = Serial(serial_port_right, 115200, timeout=0)

        # subscribe
        self._subscriber_left = self.create_subscription(
            JointTrajectoryPoint,
            'joint_trajectory_point_left',
            self.listener_callback_left,
            10
        )
        self._subscriber_right = self.create_subscription(
            JointTrajectoryPoint,
            'joint_trajectory_point_right',
            self.listener_callback_right,
            10
        )

    def listener_callback_left(self, msg: JointTrajectoryPoint):
        # TODO: send pos to esp32
        radian_positions = msg.positions
        self.get_logger().info(f"receive {radian_positions}")

        # radian to degree
        degree_positions = [math.degrees(rad) % 360 for rad in radian_positions]
        ctrl_json = {"servo_target_angles": degree_positions}
        try:
            ctrl_str = orjson.dumps(ctrl_json, option=orjson.OPT_APPEND_NEWLINE)
            self._serial_left.write(ctrl_str)
        except orjson.JSONEncodeError as error:
            self.get_logger().error(f"LEFT: Json encode error when recv message: {msg}: {error}")
            return
        # log
        self.get_logger().info(f"{ctrl_str}")

    def listener_callback_right(self, msg: JointTrajectoryPoint):
        # TODO: send pos to esp32
        radian_positions = msg.positions
        self.get_logger().info(f"receive {radian_positions}")

        # radian to degree
        degree_positions = [math.degrees(rad) % 360 for rad in radian_positions]
        ctrl_json = {"servo_target_angles": degree_positions}
        try:
            ctrl_str = orjson.dumps(ctrl_json, option=orjson.OPT_APPEND_NEWLINE)
            self._serial_right.write(ctrl_str)
        except orjson.JSONEncodeError as error:
            self.get_logger().error(f"RIGHT: Json encode error when recv message: {msg}: {error}")
            return
        # log
        self.get_logger().info(f"{ctrl_str}")


def main(args=None):
    rclpy.init(args=args)
    serial_writer = ArmSerialWriter()
    rclpy.spin(serial_writer)

    serial_writer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
