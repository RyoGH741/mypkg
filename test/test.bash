#!/bin/bash
set -e
python3 -m compileall .
colcon build --packages-select mypkg