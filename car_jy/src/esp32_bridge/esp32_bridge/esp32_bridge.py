import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import time


class Esp32BridgeNode(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_sep', 0.0)
        self.declare_parameter('wheel_radius', 0.0)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value

        self.serial_port = None
        self.last_reconnect_attempt = 0.0
        self.connect_serial()

        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.pub_status = self.create_publisher(
            String,
            '/amr/status',
            10
        )

        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.read_serial_timer_callback)

    def connect_serial(self):
        try:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
            self.serial_port = serial.Serial(self.port, self.baud, timeout=0.01)
            self.get_logger().info(f"ESP32 연결 성공: 포트 {self.port} @ {self.baud} bps")
            return True
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f"ESP32 시리얼 연결 대기 중 ({self.port}): {e}", throttle_duration_sec=5.0)
            self.serial_port = None
            return False

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        cmd_str = f"$cmd, {v:.3f}, {w:.3f}\n"
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(cmd_str.encode('utf-8'))
                self.serial_port.flush()
            except (serial.SerialException, OSError) as e:
                self.get_logger().warn(f"시리얼 전송 중 오류: {e}", throttle_duration_sec=2.0)
                self.handle_disconnection()

    def handle_disconnection(self):
        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None

    def read_serial_timer_callback(self):
        if not self.serial_port or not self.serial_port.is_open:
            now = time.time()
            if now - self.last_reconnect_attempt > 3.0:
                self.last_reconnect_attempt = now
                self.connect_serial()
            return

        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    status_msg = String()
                    status_msg.data = line
                    self.pub_status.publish(status_msg)
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f"시리얼 통신 끊김 감지 ({self.port}): {e}", throttle_duration_sec=3.0)
            self.handle_disconnection()
        except Exception as e:
            self.get_logger().warn(f"시리얼 예외 발생: {e}", throttle_duration_sec=3.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Esp32BridgeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            if hasattr(node, 'serial_port') and node.serial_port is not None and node.serial_port.is_open:
                try:
                    node.serial_port.close()
                except Exception:
                    pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
