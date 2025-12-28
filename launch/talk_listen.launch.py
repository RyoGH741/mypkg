#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Ryoichi Sakamaki
# SPDX-License-Identifier: BSD-3-Clause
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():

    talker = launch_ros.actions.Node(
        package='mypkg',#パッケージの名前を指定
        executable='mic_freq_pub',#実行するファイルの指定
        )
    listener = launch_ros.actions.Node(
        package='mypkg',
        executable='listener',
        output='screen'#ログを端末に出すための設定
        )
    piano = launch_ros.actions.Node(
        package='mypkg',
        executable='piano',
        output='screen'
        )

    return launch.LaunchDescription([talker, listener, piano])         
