#!/usr/bin/env bash

cd "$(dirname "$0")"
cd ../../
(colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug --symlink-install && ./build/ornis/ornis )
cd ornis
rm compile_commands.json
cp ../build/compile_commands.json .
