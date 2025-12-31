#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Ryoichi Sakamaki
# SPDX-License-Identifier: BSD-3-Clause
import launch
import launch_ros.actions


def generate_launch_description():

    mic_freq_pub = launch_ros.actions.Node(
        package='mypkg',
        executable='mic_freq_pub',
        output='screen'
        )
    tuner_node = launch_ros.actions.Node(
        package='mypkg',
        executable='tuner_node',
        output='screen'
        )
    draw_piano = launch_ros.actions.Node(
        package='mypkg',
        executable='draw_piano',
        output='screen'
        )

    return launch.LaunchDescription([mic_freq_pub, tuner_node, draw_piano])         
