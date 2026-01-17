#!/bin/bash
# SPDX-FileCopyrightText: 2025 Ryoichi Sakamaki
# SPDX-License-Identifier: GPL-3.0-only

dir=~
[ "$1" != "" ] && dir="$1"

ng () {
	echo ${1}行目が失敗
	res=1
}

cd $dir/ros2_ws
colcon build
source /opt/ros/humble/setup.bash
source $dir/ros2_ws/install/setup.bash

timeout 10 ros2 launch mypkg mic_to_piano.launch.py > /tmp/mypkg.log

res=0

#tuner_node に freq が送られているかの確認
cat /tmp/mypkg.log | grep -q 'freq : '
[ "$?" = 0 ] || ng "$((LINENO - 1))"

#deaw_piano に note_info が送られているかの確認
cat /tmp/mypkg.log | grep -q 'note_info : '
[ "$?" = 0 ] || ng "$((LINENO - 1))"

[ "${res}" = 0 ] && echo OK

exit $res