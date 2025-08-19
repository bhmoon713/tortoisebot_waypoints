# tortoisebot_waypoints — Testing Instructions

## Start simulation (Terminal 1)
```bash
source /opt/ros/noetic/setup.bash
source ~/simulation_ws/devel/setup.bash
roslaunch tortoisebot_gazebo tortoisebot_playground.launch



## Launch the Waypoints Action Server for ROS1: (Terminal 2)
```bash
source /opt/ros/noetic/setup.bash
cd ~/simulation_ws && catkin_make && source devel/setup.bash
rosrun tortoisebot_waypoints tortoisebot_action_server.py
```

## Run test(Terminal 3) : Pass
```bash
source /opt/ros/noetic/setup.bash
cd ~/simulation_ws && catkin_make && source devel/setup.bash
rostest tortoisebot_waypoints waypoints_test_pass.test --reuse-master
```

## Run test(Terminal 3) : Fail
```bash
source /opt/ros/noetic/setup.bash
cd ~/simulation_ws && catkin_make && source devel/setup.bash
rostest tortoisebot_waypoints waypoints_test_fail.test --reuse-master
```
##or you can play with any number numbers (Terminal 3)
```bash
rostest tortoisebot_waypoints waypoints_test.test --reuse-master
```
