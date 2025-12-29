#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Ryoichi Sakamaki
# SPDX-License-Identifier: BSD-3-Clause
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import ChannelFloat32
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class DrawPiano(Node):
    def __init__(self):
        super().__init__("draw_piano")
        self.sub = self.create_subscription(ChannelFloat32, "note_info", self.cb, 10)

        self.get_logger().info("draw_piano started")

        #描画設定
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 4))
        self.ax.set_xlim(5, 57)
        self.ax.set_ylim(-1.4, 1.6)
        self.ax.axis("off")
        self.all_keys = {}
        self.create_all_keys()
        # テキストオブジェクト（書き換え用）
        self.info_text = self.ax.text(26, 1.2, "Waiting for sound...", ha="center", fontsize=14, fontweight="bold")
        self.meter_text = self.ax.text(26, -0.8, "", ha="center", family="monospace", fontsize=12)
        plt.show(block=False)
    
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
                x += 1

        #黒鍵の描画
        x2 = 0
        offsets = [0,1,3,4,5]

        for octave in range(0, 8):
            for j, i in enumerate(black):
                black_name = f"{i}{octave}"
                rect = patches.Rectangle((x2 + offsets[j] + 0.5, 0.4), 0.8, 0.6, facecolor="black", edgecolor="black", zorder=2)
                self.ax.add_patch(rect)
                self.all_keys[black_name] = rect
            x2 += 7

    def cb(self, note_info_msg):
# 1. 全ての色をリセット
        for name, rect in self.all_keys.items():
            # 名前に 's' (sharp) が含まれるかどうかで色を決める
            default_color = "black" if ("s" in name or "B" in name) else "white"
            rect.set_facecolor(default_color)
    
        # 該当する音を赤くする
        target_note = note_info_msg.name
        if target_note in self.all_keys:
            self.all_keys[target_note].set_facecolor("red")

        # メーターとテキストの更新
        diff = note_info_msg.values[0]
        base = note_info_msg.values[1]
        
        if diff < -0.5: meter = "[ < |     ] low"
        elif diff > 0.5: meter = "[     | > ] high"
        else: meter = "[   |OK|   ] just"

        self.info_text.set_text(f"Note: {target_note}  ({base}Hz)")
        self.meter_text.set_text(f"{meter}\nDiff: {diff:+.2f} Hz")
        #再描画
        self.fig.canvas.flush_events()


def main():
    rclpy.init()
    node = DrawPiano()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()