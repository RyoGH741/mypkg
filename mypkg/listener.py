import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class Tuner(Node):

    def __init__(self):
        super().__init__("tuner")
        self.sub = self.create_subscription(Float32, "mic_freq", self.cb, 10)
        self.pub = self.create_publisher(String, "note_info", 10)

        self.onmei_list = ["A", "B", "H", "C", "Ds", "D", "Es", "E", "F", "Gs", "G", "As"]

    def heikintitsu(self, n):
        return 442 * 2 ** (1 / 12 * (n-49))
    
    def cb(self, msg):
        freq = msg.data
        number = 0
        onmei = "Undefind"
        difference = 0.0

        for i in range(1, 89):
            if ((i+8) % 12 == 0):
                number += 1

            if (self.heikinritu(i - 0.5) <= freq) and (freq < self.heikinritu(i + 0.5)):
                onmei = self.onmei_list[i % 12 - 1]
                difference = round(self.heikinritu(i) - freq, 3)
                break
            elif(freq < self.heikinritu(i - 0.5)):
                onmei = "Undefined"
                difference = round(self.heikinritu(i) - freq, 3)
                break
            elif(self.heikinritu(i + 0.5) <= freq ) and (88 == i):
                onmei = "Undefined"
                difference = round(self.heikinritu(i) - freq, 3)
                break

        output = f"{onmei}{number} {difference}"
        msg = String()
        msg.data = output
        self.publisher.publish(msg)



def main():
    rclpy.init()
    node = Tuner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()