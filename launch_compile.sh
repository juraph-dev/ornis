#!/usr/bin/env bash
set -euo pipefail

# (cd build; /usr/local/bin/cmake -DCMAKE_BUILD_TYPE=DEBUG -DCMAKE_EXPORT_COMPILE_COMMANDS=1 .. && make && ./Rostui )
(cd build; cmake -DCMAKE_BUILD_TYPE=DEBUG -DCMAKE_EXPORT_COMPILE_COMMANDS=1 .. && make && clear && ./Rostui )
rm compile_commands.json
cp build/compile_commands.json .
