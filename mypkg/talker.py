import rclpy#ROS 2のクライアントのためのライブラリ
from rclpy.node import Node#ノードを実装するためのNodeクラス（クラスは第10回で)
from std_msgs.msg import Int16#通信の型（16ビットの符号付き整）

rclpy.init()
node = Node("talker")#ノード作成（nodeという「オブジェクト」を作成）
pub = node.create_publisher(Int16, "countup", 10)#パブリッシャのオブジェクト作成
n = 0#カウント用変数


def cb():
    global n
    msg = Int16()
    msg.data = n
    pub.publish(msg)
    n += 1


def main():
    node.create_timer(0.5, cb)
    rclpy.spin(node)
