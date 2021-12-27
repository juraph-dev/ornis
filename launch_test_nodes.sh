#!/usr/bin/env bash

roscore &
(sleep 4 ; rostopic pub /test1 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test2 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test3 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test4 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test5 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test6 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test7 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test8 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test9 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test10 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test11 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test12 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test13 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test14 std_msgs/Empty "{}") > /dev/null 2>&1 
