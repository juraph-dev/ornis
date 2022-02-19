#! /bin/sh
cd ../../
rm -r build/*
rm -r install/*
colcon build --cmake-clean-cache --cmake-args " -DFIX_INCLUDES=ON" " -DCHECK_INCLUDES=ON" " -DUSE_POC=OFF"  " -DCMAKE_CXX_OUTPUT_EXTENSION_REPLACE=ON" " -DCMAKE_BUILD_TYPE=Debug" " -DCMAKE_EXPORT_COMPILE_COMMANDS=ON" --symlink-install --event-handlers console_direct+
rm ornis/compile_commands.json
cp build/compile_commands.json ornis/compile_commands.json
cd ../../
# Until I figure out how to get the ament
# linters to ignore the external/ directory, don't use colcon test
# colcon test --event-handlers console_cohesion+
