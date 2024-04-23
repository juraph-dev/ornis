#! /bin/bash
set -euo pipefail
cd "$(dirname "$0")"
cd ../..
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug  -DCMAKE_CXX_OUTPUT_EXTENSION_REPLACE=ON  --symlink-install --event-handlers console_direct+
cd ornis
rm compile_commands.json
cp ../build/compile_commands.json .
