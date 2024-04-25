import copy
import curses
import math
from robot_control.env import *
import rclpy
from rclpy.node import Node
import threading
import time
from trajectory_msgs.msg import JointTrajectoryPoint


def clamp(value, min_value, max_value):
    """限制值在一定範圍內"""
    return max(min_value, min(value, max_value))


class ArmKeyboardController(Node):
    def __init__(self, stdscr):
        super().__init__('arm_keyboard')
        self._joint_pos_left_min = [math.radians(x) for x in [0, 0, 0, 60, 10]]
        self._joint_pos_left_max = [math.radians(x) for x in [180, 90, 180, 180, 70]]
        self._joint_pos_right_min = [math.radians(x) for x in [0, 80, 0, 0, 0]]
        self._joint_pos_right_max = [math.radians(x) for x in [180, 180, 180, 120, 70]]
        self._init_joint_pos_left = [math.radians(x) for x in [170, 10, 100, 150, 30]]
        self._init_joint_pos_right = [math.radians(x) for x in [10, 170, 80, 30, 30]]
        self.joint_pos_left = copy.deepcopy(self._init_joint_pos_left)
        self.joint_pos_right = copy.deepcopy(self._init_joint_pos_right)
        self.rotate_angle = math.radians(10.0)  # 控制機械手臂每次移動的角度
        self.rotate_speed = 15

        self.joint_trajectory_publisher_left_ = self.create_publisher(
            JointTrajectoryPoint,
            'joint_trajectory_point_left',
            10
        )
        self.joint_trajectory_publisher_right_ = self.create_publisher(
            JointTrajectoryPoint,
            'joint_trajectory_point_right',
            10
        )

        self.stdscr = stdscr
        curses.noecho()
        curses.raw()
        self.stdscr.keypad(False)
        self.key_in_count = 0

    def pub_arm(self):
        msg_left = JointTrajectoryPoint()
        msg_left.positions = [float(pos) for pos in self.joint_pos_left]
        msg_left.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]  # Replace with actual desired velocities
        msg_right = copy.deepcopy(msg_left)
        msg_right.positions = [float(pos) for pos in self.joint_pos_right]
        self.joint_trajectory_publisher_left_.publish(msg_left)
        self.joint_trajectory_publisher_right_.publish(msg_right)

    def run(self):
        self.stdscr.nodelay(True)
        try:
            while rclpy.ok():
                c = self.stdscr.getch()

                # Check if a key was actually pressed
                if c != curses.ERR:
                    self.key_in_count += 1
                    self.print_basic_info(c)
                    if c == ord('w'):
                        self.handle_key_w()
                    elif c == ord('s'):
                        self.handle_key_s()
                    elif c == ord('a'):
                        self.handle_key_a()
                    elif c == ord('d'):
                        self.handle_key_d()
                    elif c == ord('z'):
                        self.handle_key_z()
                    elif c == ord('x'):
                        self.handle_key_x()
                    elif c == ord('q'):
                        self.handle_key_q()
                    elif c == ord('e'):
                        self.handle_key_e()
                    elif c == ord('c'):
                        self.handle_key_c()
                    elif c == ord('v'):
                        self.handle_key_v()
                    elif c == ord('i'):
                        self.handle_key_i()
                    elif c == ord('k'):
                        self.handle_key_k()
                    elif c == ord('j'):
                        self.handle_key_j()
                    elif c == ord('l'):
                        self.handle_key_l()
                    elif c == ord(','):
                        self.handle_key_comma()
                    elif c == ord('.'):
                        self.handle_key_period()
                    elif c == ord('u'):
                        self.handle_key_u()
                    elif c == ord('o'):
                        self.handle_key_o()
                    elif c == ord('n'):
                        self.handle_key_n()
                    elif c == ord('m'):
                        self.handle_key_m()
                    elif c == ord('b'):
                        self.handle_key_b()
                    elif c == ord('2'):
                        self.handle_key_2()
                    elif c == ord('1'):  # Exit on '1'
                        self.joint_pos_left = self._init_joint_pos_left
                        self.joint_pos_right = self._init_joint_pos_right
                        break
                    self.pub_arm()
                    print()
                else:
                    self.print_basic_info(ord(' '))
                    time.sleep(0.01)

        finally:
            curses.endwin()

    def print_basic_info(self, key):
        # Clear the screen
        self.stdscr.clear()

        self.stdscr.move(0, 0)
        # Print a string at the current cursor position
        self.stdscr.addstr(f"{self.key_in_count:5d} Key '{chr(key)}' pressed!")

        # show receive data
        self.stdscr.move(1, 0)
        self.stdscr.addstr(f"Left arm pos:\t{[round(math.degrees(x), 3) for x in self.joint_pos_left]}")
        self.stdscr.move(2, 0)
        self.stdscr.addstr(f"Right arm pos:\t{[round(math.degrees(x), 3) for x in self.joint_pos_right]}")
        self.stdscr.move(3, 0)

        # self.get_logger().debug(f"{self.key_in_count:5d} Key '{chr(key)}' pressed!")

    def angle_calc(self, is_left: bool, idx: int, is_add: bool):
        if is_left:
            self.joint_pos_left[idx] = clamp(
                self.joint_pos_left[idx] + self.rotate_angle if is_add else self.joint_pos_left[
                                                                                idx] - self.rotate_angle,
                self._joint_pos_left_min[idx],
                self._joint_pos_left_max[idx],
            )
        else:
            self.joint_pos_right[idx] = clamp(
                self.joint_pos_right[idx] + self.rotate_angle if is_add else self.joint_pos_right[
                                                                                 idx] - self.rotate_angle,
                self._joint_pos_right_min[idx],
                self._joint_pos_right_max[idx],
            )

    # LEFT
    def handle_key_w(self):
        self.stdscr.addstr(f"Left J1 up")
        self.angle_calc(is_left=True, idx=0, is_add=False)
        pass

    def handle_key_s(self):
        self.stdscr.addstr(f"Left J1 down")
        self.angle_calc(is_left=True, idx=0, is_add=True)
        pass

    def handle_key_a(self):
        self.stdscr.addstr(f"Left J2 up")
        self.angle_calc(is_left=True, idx=1, is_add=True)
        pass

    def handle_key_d(self):
        self.stdscr.addstr(f"Left J2 down")
        self.angle_calc(is_left=True, idx=1, is_add=False)
        pass

    def handle_key_z(self):
        self.stdscr.addstr(f"Left J3 counterclockwise")
        self.angle_calc(is_left=True, idx=2, is_add=False)
        pass

    def handle_key_x(self):
        self.stdscr.addstr(f"Left J3 clockwise")
        self.angle_calc(is_left=True, idx=2, is_add=True)
        pass

    def handle_key_q(self):
        self.stdscr.addstr(f"Left J4 left")
        self.angle_calc(is_left=True, idx=3, is_add=False)
        pass

    def handle_key_e(self):
        self.stdscr.addstr(f"Left J4 right")
        self.angle_calc(is_left=True, idx=3, is_add=True)
        pass

    def handle_key_c(self):
        self.stdscr.addstr(f"Left J5 release")
        self.angle_calc(is_left=True, idx=4, is_add=True)
        pass

    def handle_key_v(self):
        self.stdscr.addstr(f"Left J5 catch")
        self.angle_calc(is_left=True, idx=4, is_add=False)
        pass

    # RIGHT
    def handle_key_i(self):
        self.stdscr.addstr(f"Right J1 up")
        self.angle_calc(is_left=False, idx=0, is_add=True)
        pass

    def handle_key_k(self):
        self.stdscr.addstr(f"Right J1 down")
        self.angle_calc(is_left=False, idx=0, is_add=False)
        pass

    def handle_key_l(self):
        self.stdscr.addstr(f"Right J2 up")
        self.angle_calc(is_left=False, idx=1, is_add=False)
        pass

    def handle_key_j(self):
        self.stdscr.addstr(f"Right J2 down")
        self.angle_calc(is_left=False, idx=1, is_add=True)
        pass

    def handle_key_comma(self):
        self.stdscr.addstr(f"Right J3 counterclockwise")
        self.angle_calc(is_left=False, idx=2, is_add=False)
        pass

    def handle_key_period(self):
        self.stdscr.addstr(f"Right J3 clockwise")
        self.angle_calc(is_left=False, idx=2, is_add=True)
        pass

    def handle_key_u(self):
        self.stdscr.addstr(f"Right J4 left")
        self.angle_calc(is_left=False, idx=3, is_add=True)
        pass

    def handle_key_o(self):
        self.stdscr.addstr(f"Right J4 right")
        self.angle_calc(is_left=False, idx=3, is_add=False)
        pass

    def handle_key_m(self):
        self.stdscr.addstr(f"Right J5 catch")
        self.angle_calc(is_left=False, idx=4, is_add=False)
        pass

    def handle_key_n(self):
        self.stdscr.addstr(f"Right J5 release")
        self.angle_calc(is_left=False, idx=4, is_add=True)
        pass

    def handle_key_b(self):
        # Reset to initial position
        self.stdscr.addstr(f"Reset")
        self.joint_pos_left = copy.deepcopy(self._init_joint_pos_left)
        self.joint_pos_right = copy.deepcopy(self._init_joint_pos_right)

    def handle_key_2(self):
        self.wave_hand_macro()

    def wave_hand_macro(self):
        interval = 0.2
        # Reset
        self.handle_key_b()
        self.pub_arm()
        # Raise hands
        self.joint_pos_left = [math.radians(x) for x in [90, 10, 90, 90, 30]]
        self.joint_pos_right = [math.radians(x) for x in [80, 170, 90, 90, 30]]
        self.pub_arm()
        time.sleep(interval)
        while True:
            # Wave left
            self.joint_pos_left = [math.radians(x) for x in [90, 10, 30, 90, 30]]
            self.joint_pos_right = [math.radians(x) for x in [80, 170, 30, 90, 30]]
            self.pub_arm()
            time.sleep(interval)
            # Wave right
            self.joint_pos_left = [math.radians(x) for x in [90, 10, 150, 90, 30]]
            self.joint_pos_right = [math.radians(x) for x in [80, 170, 150, 90, 30]]
            self.pub_arm()
            time.sleep(interval)

            c = self.stdscr.getch()
            # Check if a key was actually pressed
            if c != curses.ERR:
                self.key_in_count += 1
                self.print_basic_info(c)
                if c == ord('1'):  # Exit on '1'
                    self.handle_key_b()
                    self.pub_arm()
                    break
                print()
            else:
                self.print_basic_info(ord(' '))
                time.sleep(0.01)


# ... Rest of your code, e.g. initializing rclpy and running the node
def main(args=None):
    rclpy.init(args=args)
    stdscr = curses.initscr()
    node = ArmKeyboardController(stdscr)

    # Spin the node in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    finally:
        curses.endwin()
        node.get_logger().info(f'Quit keyboard!')
        rclpy.shutdown()
        spin_thread.join()  # Ensure the spin thread is cleanly stopped


if __name__ == '__main__':
    main()
