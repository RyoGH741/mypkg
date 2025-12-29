#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Ryoichi Sakamaki
# SPDX-License-Identifier: BSD-3-Clause
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():

    mic_freq_pub = launch_ros.actions.Node(
        package='mypkg',#パッケージの名前を指定
        executable='mic_freq_pub',#実行するファイルの指定
        )
    tuner_node = launch_ros.actions.Node(
        package='mypkg',
        executable='tuner_node',
        output='screen'#ログを端末に出すための設定
        )
    draw_piano = launch_ros.actions.Node(
        package='mypkg',
        executable='draw_piano',
        output='screen'
        )

    return launch.LaunchDescription([mic_freq_pub, tuner_node, draw_piano])         
