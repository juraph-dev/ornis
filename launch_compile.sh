#!/usr/bin/env bash

(colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug --symlink-install && ./build/rostui/Rostui )
rm compile_commands.json
cp build/compile_commands.json .
