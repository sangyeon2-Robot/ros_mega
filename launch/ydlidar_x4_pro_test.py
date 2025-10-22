#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

class ScanToPolar(Node):
    def __init__(self):
        super().__init__("scan_to_polar")
        # 퍼블리셔 QoS: 구독자들이 쓰기 쉽도록 기본값(volatile/best_effort)
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_angles = self.create_publisher(Float32MultiArray, "scan_angles_deg", qos_pub)
        self.pub_ranges = self.create_publisher(Float32MultiArray, "scan_ranges_m", qos_pub)
        self.pub_pairs  = self.create_publisher(Float32MultiArray, "scan_polar_pairs", qos_pub)

        # 구독 QoS: 드라이버가 BEST_EFFORT 이므로 동일하게 맞춤
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(LaserScan, "/scan", self.cb, qos_sub)

        # 파라미터(옵션)
        self.declare_parameter("drop_invalid", True)   # inf/NaN/0.0 제거
        self.declare_parameter("deg", True)            # 각도를 도(deg)로
        self.declare_parameter("downsample", 1)        # N개마다 1개 샘플(부하 줄이기)

    def cb(self, msg: LaserScan):
        drop_invalid = self.get_parameter("drop_invalid").get_parameter_value().bool_value
        use_deg      = self.get_parameter("deg").get_parameter_value().bool_value
        downsample   = max(1, self.get_parameter("downsample").get_parameter_value().integer_value)

        angles = []
        ranges = []
        # 계산
        for i in range(0, len(msg.ranges), downsample):
            r = msg.ranges[i]
            if drop_invalid:
                if math.isinf(r) or math.isnan(r) or r <= 0.0:
                    continue
            theta = msg.angle_min + i * msg.angle_increment
            if use_deg:
                theta = theta * 180.0 / math.pi
            angles.append(float(theta))
            ranges.append(float(r))

        # 퍼블리시
        if angles:
            msg_angles = Float32MultiArray(data=angles)
            msg_ranges = Float32MultiArray(data=ranges)
            pairs = []
            pairs.extend([v for pair in zip(angles, ranges) for v in pair])
            msg_pairs  = Float32MultiArray(data=pairs)

            self.pub_angles.publish(msg_angles)
            self.pub_ranges.publish(msg_ranges)
            self.pub_pairs.publish(msg_pairs)

def main():
    rclpy.init()
    node = ScanToPolar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
