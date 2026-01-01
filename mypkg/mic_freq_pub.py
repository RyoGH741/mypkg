#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Ryoichi Sakamaki
# SPDX-License-Identifier: GPL-3.0-only
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sounddevice as sd
import numpy as np
import aubio


class MicFreqPub(Node):

    def __init__(self):
        super().__init__("mic_freq_pub")
        self.pub = self.create_publisher(Float32, "mic_freq", 10)

        self.sr = 44100    #サンプリング周波数
        self.bs = 1024 * 2        #周波数解析範囲
        self.device = None        #デバイス番号、任意に変更
        #aubioの設定
        self.pitch_o = aubio.pitch("yin", self.bs * 2, self.bs, self.sr)
        self.pitch_o.set_unit("Hz")
        
        #音声取得の設定
        try:
            self.stream = sd.InputStream(channels=1, samplerate=self.sr,
                                        blocksize=self.bs, callback=self.cb,
                                            device=self.device)
            self.stream.start()
            self.get_logger().info("audio stream started")
        except Exception as e:
            self.get_logger().error(f"faild to start audio stream: {e}")

    def cb(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f"Sounddevice status: {status}")
        
        #YIN計算
        audio = indata[:, 0].astype(np.float32)
        freq = float(self.pitch_o(audio)[0])

        #送信
        msg = Float32()
        msg.data = freq
        self.pub.publish(msg)
        self.get_logger().info(f"{freq}")
        
def main():
    rclpy.init()
    node = MicFreqPub()
    rclpy.spin(node)
    node.stream.stop()
    node.stream.close()
    node.destroy_node()
    rclpy.shutdown()