#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Ryoichi Sakamaki
# SPDX-License-Identifier: GPL-3.0-only
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import ChannelFloat32
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class DrawPiano(Node):
    def __init__(self):
        super().__init__("draw_piano")
        self.sub = self.create_subscription(ChannelFloat32, "note_info", self.cb, 10)

        #描画設定
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 4))
        self.ax.set_xlim(5, 57)
        self.ax.set_ylim(-1.4, 1.6)
        self.ax.axis("off")
        self.all_keys = {}
        self.create_all_keys()
        self.base_freq_text = self.ax.text(28, 1.2, "", ha="center", fontsize=18, fontweight="bold")
        self.meter_text = self.ax.text(20, -0.8, "", ha="left", fontsize=18, family="monospace", fontweight = "bold")
        plt.show(block=False)
    
    #鍵盤の描画
    def create_all_keys(self):
        white = ["C", "D", "E", "F", "G", "A", "H"]
        black = ["Ds", "Es", "Gs", "As", "B"]

        #白鍵の描画
        x = 0
        for octave in range(0, 9):
            for i in white:
                white_name = f"{i}{octave}"
                rect = patches.Rectangle((x, 0), 1, 1, edgecolor="black", facecolor="white")
                self.ax.add_patch(rect)
                self.all_keys[white_name] = rect
                #白鍵の下にCを書く
                if i == "C" and 1 <= octave <= 8:
                    self.ax.text(x + 0.5, -0.1, "C", fontsize=10, ha="center")
                x += 1

        #黒鍵の描画
        x2 = 0
        pitch = [0,1,3,4,5]
        for octave in range(0, 8):
            for j, i in enumerate(black):
                if not (octave == 0 and j == 3):
                    black_name = f"{i}{octave}"
                    rect = patches.Rectangle((x2 + pitch[j] + 0.5, 0.4), 0.8, 0.6, facecolor="black", edgecolor="black", zorder=2)
                    self.ax.add_patch(rect)
                    self.all_keys[black_name] = rect
            x2 += 7

    def cb(self, msg):
        self.get_logger().info(f"from tuner_node to draw_piano : {msg.name} {msg.values}")

        #全ての色をリセット
        for name, rect in self.all_keys.items():
            default_color = "black" if ("s" in name or "B" in name) else "white"
            rect.set_facecolor(default_color)
    
        #該当する音を赤くする
        target_note = msg.name
        if target_note in self.all_keys:
            self.all_keys[target_note].set_facecolor("red")

        #テキストの更新
        diff = round(msg.values[0], 2)
        base = round(msg.values[1], 2)
        
        if diff < -0.5:
            meter = " low [ <  |    ]"
        elif diff > 0.5:
            meter = "     [    |  > ] high"
        else:
            meter = "     [   |OK|  ]"

        self.base_freq_text.set_text(f"{target_note} : {base}Hz")
        self.meter_text.set_text(f"{meter}\nDiff: {diff}Hz")
        
        #再描画
        self.fig.canvas.flush_events()


def main():
    rclpy.init()
    node = DrawPiano()
    rclpy.spin(node)
    plt.close('all')
    node.destroy_node()
    rclpy.shutdown()