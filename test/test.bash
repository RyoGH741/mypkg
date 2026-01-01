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
python3 -m sounddevice
cat /tmp/mypkg.log
ng () {
	res=1
}
res=0
cat /tmp/mypkg.log | grep -q 'audio stream started'
[ "$?" = 0 ] || [ res=1 ]
cat /tmp/mypkg.log | grep -q 'from mic_freq_pub to tuner_node'
[ "$?" = 0 ] || [ res=1 ]
cat /tmp/mypkg.log | grep -q 'from tuner_node to draw_piano'
[ "$?" = 0 ] || [ res=1 ]

[ "${res}" = 0 ] && echo OK

exit $res
