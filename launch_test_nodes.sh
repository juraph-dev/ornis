#!/usr/bin/env bash

(sleep 4 ; ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}" -n test1) > /dev/null 2>&1 &
(sleep 4 ; ros2 topic pub /turtle2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}" -n test2) > /dev/null 2>&1 &
(sleep 4 ; ros2 topic pub /turtle3/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}" -n test3) > /dev/null 2>&1 &
(sleep 4 ; ros2 topic pub /turtle4/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}" -n test4) > /dev/null 2>&1 &
(sleep 4 ; ros2 topic pub /turtle5/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}" -n test5) > /dev/null 2>&1 
# (sleep 4 ; ros2 topic pub /test6 std_msgs/Float32 123.3) > /dev/null 2>&1 &
# (sleep 4 ; ros2 topic pub /test7 std_msgs/Float32 123.3) > /dev/null 2>&1 &
# (sleep 4 ; ros2 topic pub /test8 std_msgs/Float32 123.3) > /dev/null 2>&1 &
# (sleep 4 ; ros2 topic pub /test9 std_msgs/Float32 123.3) > /dev/null 2>&1 &
# (sleep 4 ; ros2 topic pub /test10 std_msgs/Int 5) > /dev/null 2>&1 &
# (sleep 4 ; ros2 topic pub /test11 std_msgs/Int 5) > /dev/null 2>&1 &
# (sleep 4 ; ros2 topic pub /test12 std_msgs/Int 5) > /dev/null 2>&1 &
# (sleep 4 ; ros2 topic pub /test13 std_msgs/Int 5) > /dev/null 2>&1 &
# (sleep 4 ; ros2 topic pub /test14 std_msgs/Int 5) > /dev/null 2>&1 
