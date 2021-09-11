#!/usr/bin/env bash
set -euo pipefail

(cd build; cmake -DCMAKE_BUILD_TYPE=DEBUG -DCMAKE_EXPORT_COMPILE_COMMANDS=1 .. && make)
rm compile_commands.json
cp build/compile_commands.json .
# Test
echo "Launching rosnodes"
trap "kill 0" EXIT

roscore > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test1 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test2 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test3 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test4 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test5 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test6 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test7 std_msgs/Empty "{}") > /dev/null 2>&1 &
(sleep 4 ; rostopic pub /test8 std_msgs/Empty "{}") > /dev/null 2>&1 &
(cd build; ./Rostui )

wait
