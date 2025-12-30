#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Ryoichi Sakamaki
# SPDX-License-Identifier: BSD-3-Clause
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import ChannelFloat32

class Tuner(Node):

    def __init__(self):
        super().__init__("tuner")
        self.sub = self.create_subscription(Float32, "mic_freq", self.cb, 10)
        self.pub = self.create_publisher(ChannelFloat32, "note_info", 10)

        self.onmei_list = ["A", "B", "H", "C", "Ds", "D", "Es", "E", "F", "Gs", "G", "As"]

    #平均律計算のメソッド
    def heikinritu(self, n):
        return 442 * 2 ** (1 / 12 * (n-49))
    
    def cb(self, msg):
        self.get_logger().info("from mic_freq_pub to tuner_node : %f" %msg.data)#送られたか確認
        
        freq = msg.data
        number = 0
        onmei = "Undefined"

        #音名の計算
        for i in range(1, 89):
            if ((i+8) % 12 == 0):
                number += 1

            low = self.heikinritu(i - 0.5)
            high = self.heikinritu(i + 0.5)

            if (low <= freq) and (freq < high):
                onmei = self.onmei_list[i % 12 - 1]
                break
            elif(freq < low):
                break
            elif(high <= freq ) and (88 == i):
                break
        
        #周波数差の計算と基準周波数
        diff = round(self.heikinritu(i) - freq, 3)
        basefreq = round(self.heikinritu(i), 3)

        #送信
        note_info_msg = ChannelFloat32()
        note_info_msg.name = f"{onmei}{number}"
        note_info_msg.values = [diff, basefreq, freq]
        self.pub.publish(note_info_msg)

def main():
    rclpy.init()
    node = Tuner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()