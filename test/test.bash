#!/bin/bash
# SPDX-FileCopyrightText: 2025 Ryoichi Sakamaki
# SPDX-License-Identifier: GPL-3.0-only

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build
# ROS2環境設定
source /opt/ros/humble/setup.bash
source $dir/ros2_ws/install/setup.bash

timeout 10 ros2 launch mypkg mic_to_piano.launch.py > /tmp/mypkg.log
#timeout 10 ros2 run mypkg mic_freq_pub
#timeout 10 ros2 launch mypkg tuner_node > /tmp/mypkg.log
#timeout 10 ros2 launch mypkg draw_piano > /tmp/mypkg.log

cat /tmp/mypkg.log
# | grep "audio stream started"
#cat /tmp/mypkg.log | grep "from mic_freq_pub to tuner_node"
#cat /tmp/mypkg.log | grep "from tuner_node to draw_piano"
