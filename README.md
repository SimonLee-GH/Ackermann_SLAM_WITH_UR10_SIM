# Ackermann_SLAM_WITH_UR10_SIM


## Requirements
 - ROS 2 Humble
 - Gazebo Fortress


## Note 
Modify the absoulte path in the file "moveo.urdf.xacro" under the path "src/mobile_base/urdf"
## Setup and build
```
# Install Nav2 dependencies
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt-get install ros-humble-moveit

# Import source dependencies
pip3 install vcstool
vcs import --input deps.repos src

# Install rosrep dependencies
rosdep install -y -r -i  --from-paths . 

# Make sure ROS2 is sourced (assuming bash, please replace extension as needed)
source /opt/ros/humble/setup.bash

# Build
colcon build

# Make sure the app is sourced (assuming bash, please replace extension as needed)
source install/setup.bash
```

## Mobile Base Demo
```
# Launch Gazebo, RViz, and Navigation2
ros2 launch mobile_base complete_navigation.launch.py
(wait until it shows "Ready for Navigation"

```
```
# Drive the Mobile Base
ros2 run mobile_base keyboard_teleop.py
```
###  UR10 moveit2 demo

launch Ignition Gazebo simulator for UR10 

```bash
ros2 launch universal_robot_ign ur10_ign.launch.py 
```

* `joint_trajectory_controller` will be used in this demo

launch moveit2 `move_group` action server for UR10.

```bash
ros2 launch universal_robot_ign ur10_moveit2_demo.launch.py 
```

run moveit2  client node, plan to goal

```bash
ros2 run universal_robot_ign test_pose_goal.py
```

* start position <-> goal pose:  `[-0.0, 0.4, 0.6, 0.0, 0.0, 0.0]`  (loop)


### UR10 + Robotiq140 Grasp demo

* control gripper Robotiq140  to grasp object and control UR10  based on joint  position.

launch Ignition Gazebo simulator for UR10  + Robotiq140

```bash
ros2 launch universal_robot_ign ur10_robotiq140_ign.launch.py 
```

*  use  Ignition plugin `RobotiqController` to control Robotiq140.

run gripper test node to grasp stick model by closing gripper 

```bash
ros2 run universal_robot_ign test_gripper.py 
#1 : close gripper to grasp.
#0 : open gripper.
```


## References

This project is based on the works in repositories [universal_robot_ign](https://github.com/gezp/universal_robot_ign)
and [ navigation2_ignition_gazebo_example]([https://github.com/gezp/universal_robot_ign](https://github.com/art-e-fact/navigation2_ignition_gazebo_example/tree/main))

