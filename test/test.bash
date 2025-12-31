#!/bin/bash
# SPDX-FileCopyrightText: 2025 Ryoichi Sakamaki
# SPDX-License-Identifier: BSD-3-Clause

dir=~
[ "$1" != "" ] && dir="$1"

res=0

cd $dir/ros2_ws
colcon build --packages-select mypkg

# ROS2環境設定
source /opt/ros/humble/setup.bash
source $dir/ros2_ws/install/setup.bash

# --- launch実行 & ログ保存 ---
echo "🚀 Launching mic_to_piano for 10 seconds..."
timeout 5 ros2 launch mypkg mic_to_piano.launch.py > /tmp/mypkg.log 2>&1
echo "📜 Log saved to /tmp/mypkg.log"

# --- テスト1: mic_freq_pub → tuner_node ---
if grep -q "from mic_freq_pub to tuner_node" /tmp/mypkg.log; then
  echo "✅ mic_freq_pub → tuner_node OK"
else
  echo "❌ mic_freq_pub → tuner_node not found"
  res=1
fi

# --- テスト2: tuner_node → draw_piano ---
if grep -q "from tuner_node to draw_piano" /tmp/mypkg.log; then
  echo "✅ tuner_node → draw_piano OK"
else
  echo "❌ tuner_node → draw_piano not found"
  res=1
fi

# --- 結果まとめ ---
if [ "$res" = 0 ]; then
  echo "🎉 All tests passed"
else
  echo "⚠️  Some tests failed"
fi

exit $res
