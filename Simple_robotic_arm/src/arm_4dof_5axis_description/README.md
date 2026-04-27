# arm_4dof_5axis_description

A minimal Gazebo arm model built for:
- 5 axis layout (motor positions 1..5 per reference)
- 4 controllable DOF (`joint_1` to `joint_4`)
- 1 coupled axis (`joint_5` mimics `joint_4`)
- 1-DOF parallel gripper (`left_finger_joint` command, right finger mimics)

## Layout mapping
- `joint_1`: base yaw (Z)
- `joint_2`: shoulder pitch (Y)
- `joint_3`: elbow pitch (Y)
- `joint_4`: wrist rotate (X)
- `joint_5`: tool-axis rotate (X), mechanically coupled with `joint_4`

## Build
```bash
cd /home/robo/simulation_of_construction_robo
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Required runtime packages
If controller loading fails with `Package 'controller_manager' not found`,
install control stack packages first:
```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-gazebo-ros2-control
```

## Run Gazebo
```bash
ros2 launch arm_4dof_5axis_description gazebo.launch.py
```

## Send one trajectory goal
```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4], points: [{positions: [0.3, 0.5, -0.6, 0.8], time_from_start: {sec: 4}}]}}"
```

## Gripper control
Open gripper:
```bash
ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [left_finger_joint], points: [{positions: [0.020], time_from_start: {sec: 1}}]}}"
```

Close gripper:
```bash
ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [left_finger_joint], points: [{positions: [0.000], time_from_start: {sec: 1}}]}}"
```

## Pick demo notes
- `empty.world` now includes a table and a red `grasp_box`.
- Drive the arm near the box, open gripper, align fingers around the box, then close gripper.
- `grasp_box` initial pose is approximately `(x=0.40, y=0.00, z=0.245)`.

## Quick grasp sequence
1. Open gripper.
2. Move arm near the box:
```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4], points: [{positions: [0.0, -0.55, 1.15, 0.25], time_from_start: {sec: 3}}]}}"
```
3. Close gripper:
```bash
ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [left_finger_joint], points: [{positions: [0.000], time_from_start: {sec: 1}}]}}"
```
4. Lift object:
```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4], points: [{positions: [0.0, -0.30, 0.85, 0.20], time_from_start: {sec: 2}}]}}"
```
