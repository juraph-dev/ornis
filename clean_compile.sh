#! /bin/sh
cd ../
rm -r build/*
rm -r install/*
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug --symlink-install
rm rostui/compile_commands.json
cp build/compile_commands.json rostui/compile_commands.json
