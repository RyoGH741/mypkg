#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Ryoichi Sakamaki
# SPDX-License-Identifier: BSD-3-Clause
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sounddevice as sd
import numpy as np


class MicFreqPub(Node):

    def __init__(self):
        super().__init__("mic_freq_pub")
        self.pub = self.create_publisher(Float32, "mic_freq", 10)

        self.samplingr = 44100    #サンプリング周波数
        self.bs = 1024 * 4        #周波数解析範囲
        self.device = None        #デバイス番号、任意に変更
        
        #リアルタイム音声取得
        try:
            self.stream = sd.InputStream(channels=1, samplerate=self.samplingr,
                                        blocksize=self.bs, callback=self.cb,
                                            device=self.device)
            self.stream.start()
            self.get_logger().info("mic_freq_pub started")
        except Exception as e:
            self.get_logger().error(f"faild to start mic_freq_pub: {e}")

    def cb(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f"Sounddevice status: {status}")
        
        #音声データ
        audio = indata[:, 0]

        #FFT計算
        fft_data = np.fft.rfft(audio)
        fft_mag = np.abs(fft_data)

        #周波数軸生成
        freqs = np.fft.rfftfreq(len(audio), 1 / self.samplingr)

        #最大振幅を取得
        max_index = np.argmax(fft_mag)
        freq = freqs[max_index]

        #送信
        freq_msg = Float32()
        freq_msg.data = float(freq)
        self.pub.publish(freq_msg)
        

def main():
    rclpy.init()
    node = MicFreqPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stream.stop()
        node.stream.close()
        node.destroy_node()
        rclpy.shutdown()