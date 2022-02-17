#! /bin/sh
cd ..
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug --symlink-install --event-handlers console_direct+
cd rostui
rm compile_commands.json
cp ../build/compile_commands.json .
