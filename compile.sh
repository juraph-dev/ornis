#!/usr/bin/env bash
set -euo pipefail

(cd build; cmake -DCMAKE_BUILD_TYPE=DEBUG -DCMAKE_EXPORT_COMPILE_COMMANDS=1  .. && make )
cp build/compile_commands.json .
