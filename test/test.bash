#!/bin/bash
# SPDX-FileCopyrightText: 2025 Ryoichi Sakamaki
# SPDX-License-Identifier: GPL-3.0-only

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build
source /opt/ros/humble/setup.bash
source $dir/ros2_ws/install/setup.bash

timeout 10 ros2 launch mypkg mic_to_piano.launch.py > /tmp/mypkg.log
cat /tmp/mypkg.log | grep "audio stream started"
echo $?
keywords=(
  "audio stream started"
  "from mic_freq_pub to tuner_node"
  "from tuner_node to draw_piano"
)

ok=true
for kw in "${keywords[@]}"; do
  if ! grep -q "$kw" /tmp/mypkg.log; then
    echo "MISS: $kw"
    ok=false
  fi
done

$ok && echo "OK"