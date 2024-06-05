# Robot Control

## Keyboard Layout

| Exit  | 2     | 3        | 4         | 5         | 6        | 7         | 8     | 9     | 0    |
| ----- | ----- | -------- | --------- | --------- | -------- | --------- | ----- | ----- | ---- |
| LJ4 ↑ | LJ1 ↑ | LJ4 ↓    | R         | T         | Y        | RJ4 ↑     | RJ1 ↑ | RJ4 ↓ | P    |
| LJ2←  | LJ1 ↓ | LJ2→     | F         | G         | H        | RJ2←      | RJ1 ↓ | RJ2→  | ;    |
| Lj3←  | LJ3→  | LJ5 open | LJ5 close | **Reset** | RJ5 open | RJ5 close | RJ3←  | RJ3→  | /    |



## References

- Left Arm: https://github.com/otischung/robot_arm_esp32/tree/left-arm-J5-master
- Right Arm: https://github.com/otischung/robot_arm_esp32/tree/right-arm-J5-master
- Docker Image: https://github.com/otischung/pros_AI_image



## Prerequisites and Installations

 Below are the devices we need.

- Arduino ESP32 *2
- Arm-control board, e.g., Raspberry Pi, Jetson Orin Nano

1. Follow the link for the left and right arms shown above to upload Arduino C++ code to ESP32.

2. Pull the docker image into the arm-control board by the following command:

   ```
   docker pull ghcr.io/otischung/pros_ai_image:latest
   ```

3. Download this project into the arm-control board.



## Usage

1. In the arm-control board, run the robot control shell script to create the container.

2. Run `colcon build --symlink-install` to build the project "robot_control".

3. Open the arm writer, and arm keyboard inside the container.

   ```
   ros2 run robot_control arm_writer
   ros2 run robot_control arm_keyboard
   ```

- We've defined the `ttyUSB0` to be the **left** robot arm and the `ttyUSB1` to be the **right** robot arm.

- If the order of the ttyUSB ID is not like we expected, then you can pass the argument into the ROS node to specify the IDs.

  ```bash
  ros2 run robot_control arm_writer --ros-args -p left:=0 -p right:=1
  ros2 run robot_control arm_writer --ros-args -p left:=/dev/ttyACM0 -p right:=/dev/ttyUSB0
  ```

- The setup method is same for `arm_reader`.
