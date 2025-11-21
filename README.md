# ENPM700 ROS 2 Programming Assignment 4 - Working with WeBots
## Overview
The objective of this assignment is to become familiar with simulating with Webots and using a Finite State Machine (FSM) design pattern. This was done by creating a ROS2 package to simulate a roomba-style robot with a simple obstacle detection algorithm that does the following:
 - Move forward until obstacle detected within 0.5 meters
 - Turn until obstacle no longer detected, then continue moving forward
 - Alternate turn directions whenever a new obstacle is encountered
The normal simulation is run through a launch file, which allows an input argument for running the ros2 bag record. A second launch file launches a version of the robot with no sensors in an open world, to demonstrate the bag playback functionality. 
## Implementing the state machine
The FSM pattern was implemented by creating a Context class called "WalkerNode" in which the 3 machine states are defined, and an abstract State Interface class called "States" which defines the common interface used by all states. The three states are:
- FORWARD: Robot moves forward until encountering an obstacle.
- TURNLEFT: Robot turns left until no obstacle detected.
- TURNRIGHT: Robot turns right until no obstacle detected.
Through the Interface class, each class contains an update method and a transition method. The update method is responsible for executing the behavior associated with the current machine state, i.e. publishing cmd_vel messages to move forward or turn right or left. The transition method evaluates the sensor data and changes the robot state accordingly depending on the current state:
- If in FORWARD:
- If obstacle detected:
    If previously turned right, switch to TURNLEFT state
    If previously turned left, switch to TURNRIGHT state
- Else:
    - Stay in FORWARD state
- If in TURNLEFT:
- If obstacle detected:
    - Stay in TURNLEFT state
- Else:
    - Switch to FORWARD state and mark previously turned left
- If in TURNRIGHT:
- If obstacle detected:
    - stay in TURNRIGHT state
- Else:
    - Switch to FORWARD state and mark previously turned right
## Assumptions
 - Using ROS2 Humble
## Dependencies
### Standard Library
Memory
### ROS2
rclcpp
geometry_msgs
sensor_msgs
Pluginlib
### Webots
Webots_ros2_driver
### Python
launch
launch_ros
## Build / Run Steps
### Simulation
1. Clone github repository
```bash
 git clone git@github.com:dzinobile/my_webots_tutorials.git
 cd my_webots_tutorials
```
2. Source ROS2, colcon-argcomplete, and colcon_cd
```bash
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
```
3. Build package and source workspace
```bash
colcon build
source install/setup.bash
```
4. Run simulation (bag record off)
```bash
ros2 launch walker walker_launch.launch.py
```
To visualize sensors, click view > optional rendering > show DistanceSensor rays.
Press ctrl + c to exit simulation.

5. Run simulation (bag record on)
```bash
ros2 launch walker walker_launch.launch.py record:=true
```
6. Let simulation run for 15-30 seconds to record bag, then press ctrl + c and verify bag record directory has appeared in workspace.
### Bag Record playback - Topic Echo
7. Launch empty world for bag play demonstration
```bash
ros2 launch walker walker_empty.launch.py
```
An empty room with a sensorless robot should spawn. 
8. Open a new terminal and play back bag 
```bash
source install/setup.bash
ros2 bag play <bag directory>
```
Verify robot begins moving as if recieving sensor data.


