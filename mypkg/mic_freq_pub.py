import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sounddevice as sd
import numpy as np


class MicFreqPub(Node):

    def __init__(self):
        super().__init__("mic_freq_pub")
        self.pub = self.create_publisher(Float32, "mic_freq", 10)

        self.samplingr = 44100
        self.bs = 1024 * 4
        self.device = None

        self.stream = sd.InputStream(channels=1, samplerate=self.samplingr, blocksize=self.bs, callback=self.cb, device=self.device)
        self.stream.start()

    def cb(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f"Sounddevice status: {status}")
        
        audio = indata[:, 0]
        #shinpuku = np.sqrt(np.mean(audio ** 2))
        # FFT実行
        fft_data = np.fft.rfft(audio)
        fft_mag = np.abs(fft_data)

        # 周波数軸生成
        freqs = np.fft.rfftfreq(len(audio), 1 / self.samplingr)

        # 最大振幅のインデックスを取得
        max_index = np.argmax(fft_mag)
        dominant_freq = freqs[max_index]

        msg = Float32()
        msg.data = float(dominant_freq)
        self.pub.publish(msg)
        

def main():
    rclpy.init()
    node = MicFreqPub()
    rclpy.spin(node)
    node.stream.stop()
    node.stream.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()