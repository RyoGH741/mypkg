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

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 3))
        self.ax.set_xlim(5, 57)
        self.ax.set_ylim(-1.4, 1.6)
        self.ax.axis("off")
        plt.show(block=False)
    
    def cb(self, note_info_msg):
        self.update_piano(note_info_msg.name, note_info_msg.values[0])

    def update_piano(self, note ,diff):
        self.ax.clear()
        self.ax.set_xlim(5, 57)
        self.ax.set_ylim(-1.4, 1.6)
        self.ax.axis("off")

        # 全鍵の音名（A0〜C8）
        white = ["C", "D", "E", "F", "G", "A", "H"]
        black = ["Ds", "Es", "Gs", "As", "B"]
        white_all = []
        black_all = []

        for octave in range(0, 8):
            for w in white:
                white_all.append(f"{w}{octave}")
        white_all.append("C8")  # 最後のC

        for octave in range(0, 8):
            for w in black:
                black_all.append(f"{w}{octave}")

        # 白鍵を描画
        x = 0
        for i in white_all:
            color = "red" if i in note else "white"
            rect = patches.Rectangle((x, 0), 1, 1, edgecolor="black", facecolor=color)
            self.ax.add_patch(rect)
            x += 1

        # 黒鍵を描画
        x2 = [0,1,3,4,5]
        x2_all = [x + 7 * n for n in range(8) for x in x2]  
        for i, name in enumerate(black_all):
            color = "red" if name in note else "black"
            rect = patches.Rectangle((x2_all[i] + 0.5, 0.4), 0.8, 0.6, facecolor=color, edgecolor="black", zorder=2)
            self.ax.add_patch(rect)

        self.ax.text(28, 1.3, f"{note}", ha="center", va="center", fontsize=16, color="black", fontweight="bold")
        self.ax.text(28, -0.5, f"{diff}Hz", ha="center", va="center", fontsize=16, color="black", fontweight="bold")

        # ==== 簡易メーター ====
        if diff < 0:
            meter_text = "- |  0    +"
            label = "low"
        elif diff > 0:
            meter_text = "-    0  | +"
            label = "high"
        else:
            meter_text = "-   |0|   +"
            label = "just"

        self.ax.text(28, -0.8, meter_text, ha="center", va="center", fontsize=14, color="black", family="monospace")
        self.ax.text(28, -1.2, f"{label}", ha="center", va="center", fontsize=12, color="black")

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main():
    rclpy.init()
    node = DrawPiano()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

