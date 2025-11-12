#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import UInt8MultiArray

class BytePublisher(Node):
    def __init__(self):
        super().__init__('byte_publisher')

        # -------- Parameters --------
        self.declare_parameter('topic_prefix', '/io')
        self.declare_parameter('topic_count', 1)
        self.declare_parameter('start_index', 0)
        self.declare_parameter('payload_size', 1024*1024)  # bytes
        self.declare_parameter('publish_rate', 50.0)       # Hz
        self.declare_parameter('message_count', 'no')      # "no" or int-like str
        self.declare_parameter('frame_id', 'lidar')        # 의미용

        self.prefix      = str(self.get_parameter('topic_prefix').value)
        self.n_topics    = int(self.get_parameter('topic_count').value)
        self.start_index = int(self.get_parameter('start_index').value)
        self.size        = int(self.get_parameter('payload_size').value)
        self.rate_hz     = float(self.get_parameter('publish_rate').value)
        limit_raw        = str(self.get_parameter('message_count').value)
        self.msg_limit   = None if limit_raw.lower() == 'no' else int(limit_raw)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.pubs = []
        self.msgs = []
        shared_bytes = bytes(self.size)
        for i in range(self.start_index, self.start_index + self.n_topics):
            topic = f'{self.prefix}{i}'
            pub = self.create_publisher(UInt8MultiArray, topic, qos)
            msg = UInt8MultiArray()
            msg.data = shared_bytes
            self.pubs.append(pub)
            self.msgs.append(msg)

        self.seq = 0
        self.timer = self.create_timer(1.0 / max(self.rate_hz, 0.1), self._tick)

        self.get_logger().info(
            f'[{self.get_name()}] publishing {self.n_topics} topics '
            f'{self.prefix}{self.start_index}..{self.prefix}{self.start_index+self.n_topics-1}, '
            f'size={self.size}B, rate={self.rate_hz}Hz, limit={limit_raw}'
        )

    def _tick(self):
        if self.msg_limit is not None and self.seq >= self.msg_limit:
            self.get_logger().info(f'Stopping after {self.seq} messages.')
            self.timer.cancel()
            return
        for pub, msg in zip(self.pubs, self.msgs):
            pub.publish(msg)
        self.seq += 1

def main(args=None):
    rclpy.init(args=args)
    node = BytePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
