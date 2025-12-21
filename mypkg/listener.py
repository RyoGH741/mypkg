import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class Tuner(Node):

    def __init__(self):
        super().__init__("tuner")
        self.sub = self.create_subscription(Float32, "/mic/freq", self.cb, 10)

    def cb(self, msg):
        self.get_logger().info(str(msg.data))


def main():
    rclpy.init()
    node = Tuner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__mian__':
    mian()