#!/bin/bash

roslaunch mybot_gazebo t4_worldV.launch

rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.01
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 

