#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial, time

class SerialCmdVelBridge(Node):
    def __init__(self):
        super().__init__('serial_cmdvel_bridge')

        # Declare ROS parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('lin_thresh', 0.05)
        self.declare_parameter('ang_thresh', 0.10)
        self.declare_parameter('deadman_sec', 0.8)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self.lin_th = float(self.get_parameter('lin_thresh').value)
        self.ang_th = float(self.get_parameter('ang_thresh').value)
        self.deadman = float(self.get_parameter('deadman_sec').value)

        # Try to open serial port
        while True:
            try:
                self.ser = serial.Serial(port, baud, timeout=0)
                self.get_logger().info(f"Connected to serial port {port} @ {baud}")
                break
            except Exception as e:
                self.get_logger().warn(f"Failed to open {port}: {e}, retrying...")
                time.sleep(1.0)

        # State variables
        self.prev_char = None
        self.last_cmd_time = 0.0
        self.last_sent = 0.0

        # Subscribe to /cmd_vel
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_twist, 10)
        # Timer for deadman stop
        self.timer = self.create_timer(0.05, self.on_timer)

    def send_char(self, ch: bytes):
        try:
            self.ser.write(ch)
            self.last_sent = time.time()
            self.prev_char = ch
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def on_twist(self, msg: Twist):
        lin = msg.linear.x
        ang = msg.angular.z

        ch = b'p'  # default: stop
        if abs(lin) > self.lin_th or abs(ang) > self.ang_th:
            if abs(lin) >= abs(ang):
                ch = b'w' if lin > 0 else b's'
            else:
                ch = b'a' if ang > 0 else b'd'

        if ch != self.prev_char or (time.time() - self.last_sent) > 0.2:
            self.send_char(ch)
        self.last_cmd_time = time.time()

    def on_timer(self):
        if (time.time() - self.last_cmd_time) > self.deadman:
            if self.prev_char != b'p':
                self.send_char(b'p')

def main():
    rclpy.init()
    node = SerialCmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
