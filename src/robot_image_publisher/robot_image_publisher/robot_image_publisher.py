#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image  # ✅ 1. CompressedImage 대신 Image 임포트
from std_msgs.msg import Header
import numpy as np
import cv2, time

class RobotImagePublisher(Node):
    def __init__(self):
        super().__init__('robot_image_publisher')

        # ---------- Parameters ----------
        self.declare_parameter('robot_index', 0)
        self.declare_parameter('publish_rate', 2.0)      # Hz
        self.declare_parameter('message_count', "no")    # "no" or int-like str
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('pattern', 'moving')      # moving | grid | noise | solid
        self.declare_parameter('frame_id', 'camera')

        self.idx           = int(self.get_parameter('robot_index').value)
        self.rate_hz       = float(self.get_parameter('publish_rate').value)
        self.msg_limit_raw = str(self.get_parameter('message_count').value)
        self.width         = int(self.get_parameter('width').value)
        self.height        = int(self.get_parameter('height').value)
        self.jpeg_quality  = int(self.get_parameter('jpeg_quality').value) # (압축 안해서 안쓰임)
        self.pattern       = str(self.get_parameter('pattern').value)
        self.frame_id      = str(self.get_parameter('frame_id').value)

        self.msg_limit = None if self.msg_limit_raw.lower() == "no" else int(self.msg_limit_raw)
        self.seq = 0
        self.t = 0.0

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # ✅ 2. Publisher 생성자 수정
        topic = f"/robotimage{self.idx}/raw"  # 토픽 이름 변경
        self.pub = self.create_publisher(Image, topic, qos) # Image 타입으로 변경

        self.timer = self.create_timer(1.0 / max(self.rate_hz, 0.1), self._on_timer)
        self.get_logger().info(
            f"[img_pub_{self.idx}] → {topic} @ {self.rate_hz}Hz, "
            f"{self.width}x{self.height}, RAW IMAGE, pattern={self.pattern}, " # Q= 제거
            f"limit={self.msg_limit_raw}"
        )

    # ---------- Image generators ----------
    def _gen_image(self):
        if self.pattern == 'grid':
            img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            step = max(min(self.width, self.height) // 20, 10)
            img[:, ::step] = 255; img[::step, :] = 255
            return img
        elif self.pattern == 'noise':
            return (np.random.rand(self.height, self.width, 3) * 255).astype(np.uint8)
        elif self.pattern == 'solid':
            color = (int((self.t*30) % 255), 100, 200)
            img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            img[:] = color
            return img
        else:  # moving
            img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            # moving bar
            w = self.width // 10
            x = int((self.t*120) % (self.width + w)) - w
            x0 = max(0, x); x1 = min(self.width, x + w)
            if x1 > x0:
                img[:, x0:x1] = (0, 255, 255)
            # moving circle
            cx = int((self.t*80) % self.width)
            cy = self.height//2
            cv2.circle(img, (cx, cy), min(self.height//6, self.width//6), (255, 128, 0), -1)
            return img

    # ✅ 3. _on_timer 함수 수정 (핵심)
    def _on_timer(self):
        if self.msg_limit is not None and self.seq >= self.msg_limit:
            self.get_logger().info(f"Reached message_count={self.msg_limit}. Stopping timer.")
            self.timer.cancel()
            return

        img = self._gen_image() # (height, width, 3) 크기의 NumPy 배열
        
        # ❌ JPEG 압축 코드 (cv2.imencode) 삭제 ❌
        
        # Compose message
        now = self.get_clock().now().to_msg()
        msg = Image() # CompressedImage 대신 Image 메시지 생성
        msg.header = Header()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        
        # Raw 이미지 정보 입력
        msg.height = self.height
        msg.width = self.width
        msg.encoding = "bgr8"  # OpenCV의 기본 컬러 순서 (3 채널, 8 비트)
        msg.is_bigendian = False
        msg.step = self.width * 3 # 1줄(row)의 용량 (가로 * 3채널)
        msg.data = img.tobytes() # 쌩 NumPy 배열을 바이트로 변환

        self.pub.publish(msg)
        self.seq += 1
        self.t += 1.0 / max(self.rate_hz, 0.1)

def main(args=None):
    rclpy.init(args=args)
    node = RobotImagePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
