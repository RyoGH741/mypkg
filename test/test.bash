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
res=0
grep -q 'audio stream started' /tmp/mypkg.log || { res=1; echo 'MISS: audio stream started'; }
grep -q 'from mic_freq_pub to tuner_node' /tmp/mypkg.log || { res=1; echo 'MISS: from mic_freq_pub to tuner_node'; }
grep -q 'from tuner_node to draw_piano' /tmp/mypkg.log || { res=1; echo 'MISS: from tuner_node to draw_piano'; }

[ "${res}" = 0 ] && echo OK

exit $res
