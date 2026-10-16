import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float32
import serial
import struct


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class StsControllerNode(Node):
    def __init__(self):
        super().__init__('sts_controller')

        # 1. 노드 파라미터 선언 (기본값 설정)
        self.declare_parameter('port', '/dev/servo_port')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('left_id', 1)            # 왼쪽 앞바퀴 ID
        self.declare_parameter('right_id', 2)           # 오른쪽 앞바퀴 ID
        self.declare_parameter('left_rear_id', 3)       # 왼쪽 뒷바퀴 ID
        self.declare_parameter('right_rear_id', 4)      # 오른쪽 뒷바퀴 ID
        self.declare_parameter('four_wheel_mode', True) # 4륜 구동 활성화 여부
        self.declare_parameter('wheel_sep', 0.30)
        self.declare_parameter('wheel_radius', 0.065)
        self.declare_parameter('wheel_mode', True)
        self.declare_parameter('invert_left', True)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('invert_angular', False)
        self.declare_parameter('scale_left', 1.0)
        self.declare_parameter('scale_right', 1.0)
        self.declare_parameter('cmd_vel_timeout', 0.4) # 초 단위 명령 수신 타임아웃
        self.declare_parameter('publish_odom_tf', True)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.left_id = self.get_parameter('left_id').value
        self.right_id = self.get_parameter('right_id').value
        self.left_rear_id = self.get_parameter('left_rear_id').value
        self.right_rear_id = self.get_parameter('right_rear_id').value
        self.four_wheel_mode = self.get_parameter('four_wheel_mode').value
        self.wheel_sep = self.get_parameter('wheel_sep').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_mode = self.get_parameter('wheel_mode').value
        self.invert_left = self.get_parameter('invert_left').value
        self.invert_right = self.get_parameter('invert_right').value
        self.invert_angular = self.get_parameter('invert_angular').value
        self.scale_left = float(self.get_parameter('scale_left').value)
        self.scale_right = float(self.get_parameter('scale_right').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.publish_odom_tf = bool(self.get_parameter('publish_odom_tf').value)

        # 2. 오도메트리 상태 및 퍼블리셔 초기화
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.last_odom_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_timer = self.create_timer(0.02, self.update_and_publish_odometry) # 50Hz

        # 3. 시리얼 포트 연결
        self.serial_port = None
        self.connect_serial()

        # 4. Wheel Mode 전환 (부팅 시 1회 실행)
        if self.wheel_mode and self.serial_port and self.serial_port.is_open:
            self.set_wheel_mode(self.left_id)
            self.set_wheel_mode(self.right_id)
            if self.four_wheel_mode:
                self.set_wheel_mode(self.left_rear_id)
                self.set_wheel_mode(self.right_rear_id)

        # 5. 서브스크립션 등록 (3가지 입력 토픽)
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.sub_left_cmd = self.create_subscription(
            Float32,
            '/left_wheel/cmd',
            self.left_wheel_callback,
            10
        )

        self.sub_right_cmd = self.create_subscription(
            Float32,
            '/right_wheel/cmd',
            self.right_wheel_callback,
            10
        )

        # 6. 안전 Watchdog 타이머 (cmd_vel이 끊기면 모터 즉시 정지)
        self.last_cmd_time = self.get_clock().now()
        self.is_stopped = True
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_callback)

    def connect_serial(self):
        try:
            self.serial_port = serial.Serial(self.port, self.baud, timeout=0.01)
            self.get_logger().info(f"STS3215 서보 포트 연결 성공: {self.port} @ {self.baud} bps")
        except Exception as e:
            self.get_logger().warn(f"STS3215 서보 포트 연결 실패: {e}")
            self.serial_port = None

    def calculate_checksum(self, packet_bytes):
        # Feetech Checksum: ~(sum(ID + Length + Instruction + Params)) & 0xFF
        s = sum(packet_bytes) & 0xFF
        return (~s) & 0xFF

    def send_write_packet(self, servo_id, reg_addr, data_bytes):
        if not self.serial_port or not self.serial_port.is_open:
            return

        length = len(data_bytes) + 3  # Inst + Reg + Data... + Chksum
        instruction = 0x03  # WRITE_DATA

        body = [servo_id, length, instruction, reg_addr] + list(data_bytes)
        chk = self.calculate_checksum(body)

        packet = bytearray([0xFF, 0xFF] + body + [chk])
        try:
            self.serial_port.write(packet)
            self.serial_port.flush()
        except Exception as e:
            self.get_logger().warn(f"서보 ID {servo_id} 패킷 전송 오류: {e}")

    def set_wheel_mode(self, servo_id):
        # EEPROM Unlock (0x37 = 0) -> Mode = 1 (Wheel Mode) -> Lock (0x37 = 1)
        self.send_write_packet(servo_id, 0x37, [0x00])
        self.send_write_packet(servo_id, 0x21, [0x01])
        self.send_write_packet(servo_id, 0x37, [0x01])
        self.get_logger().info(f"서보 ID {servo_id} Wheel Mode (EEPROM Unlock -> Set) 완료")

    def send_speed(self, servo_id, speed_val):
        val = int(speed_val)
        mag = min(abs(val), 32767)

        # Feetech STS3215 프로토콜 규격:
        # 하위 15비트(0~14): 속도 크기(Magnitude)
        # 15번째 비트(Bit 15, 0x8000 / 32768): 회전 방향 플래그
        if val < 0:
            encoded_val = (1 << 15) | mag
        else:
            encoded_val = mag

        packed_data = struct.pack('<H', encoded_val)
        self.send_write_packet(servo_id, 0x2E, packed_data)

    def send_sync_speeds(self, speeds_dict: dict):
        """
        Feetech STS3215 Sync Write (Instruction 0x83)
        모든 바퀴 서보 모터에 정확히 동일한 시점에 속도 명령을 동시 전달
        """
        if not self.serial_port or not self.serial_port.is_open:
            return

        reg_addr = 0x2E  # Goal Speed register
        data_len = 2     # 2 bytes per servo
        num_servos = len(speeds_dict)
        length = (data_len + 1) * num_servos + 4
        instruction = 0x83  # SYNC_WRITE

        body = [0xFE, length, instruction, reg_addr, data_len]
        for sid, speed_val in speeds_dict.items():
            val = int(speed_val)
            mag = min(abs(val), 32767)
            if val < 0:
                encoded_val = (1 << 15) | mag
            else:
                encoded_val = mag
            spd_l = encoded_val & 0xFF
            spd_h = (encoded_val >> 8) & 0xFF
            body.extend([sid, spd_l, spd_h])

        chk = self.calculate_checksum(body)
        packet = bytearray([0xFF, 0xFF] + body + [chk])
        try:
            self.serial_port.write(packet)
            self.serial_port.flush()
        except Exception as e:
            self.get_logger().warn(f"Sync Write 패킷 전송 오류: {e}")

    def stop_motors(self):
        """모든 바퀴 서보 모터 속도를 0으로 동시 설정"""
        self.vx = 0.0
        self.wz = 0.0
        speeds = {
            self.left_id: 0,
            self.right_id: 0
        }
        if self.four_wheel_mode:
            speeds[self.left_rear_id] = 0
            speeds[self.right_rear_id] = 0
        self.send_sync_speeds(speeds)

    def watchdog_callback(self):
        """명령 타임아웃 감시 -> 모터 자동 정지"""
        if self.is_stopped:
            return
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_vel_timeout:
            self.stop_motors()
            self.is_stopped = True
            self.get_logger().info("⚠️ [Safety Watchdog] /cmd_vel 입력 타임아웃 -> 모터 안전 자동 정지")

    def update_and_publish_odometry(self):
        """50Hz 주기 데드레커닝 오도메트리 계산 및 /odom, TF 브로드캐스트"""
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now

        if dt > 0.0:
            delta_x = (self.vx * math.cos(self.th)) * dt
            delta_y = (self.vx * math.sin(self.th)) * dt
            delta_th = self.wz * dt

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

        quat = yaw_to_quaternion(self.th)

        # 1. TF 브로드캐스트 (odom -> base_footprint)
        if self.publish_odom_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = quat
            self.tf_broadcaster.sendTransform(t)

        # 2. Odometry 메시지 발행 (/odom)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.wz

        self.odom_pub.publish(odom)

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = -msg.angular.z if self.invert_angular else msg.angular.z

        self.last_cmd_time = self.get_clock().now()

        # 속도가 0에 가까우면 즉시 정지 상태로 전환
        if abs(v) < 1e-4 and abs(w) < 1e-4:
            self.stop_motors()
            self.is_stopped = True
            return

        self.is_stopped = False
        self.vx = v
        self.wz = w

        # 차륜형 모바일 로봇 역운동학 식 적용
        L = self.wheel_sep
        R = self.wheel_radius

        v_left_rad = (2.0 * v - w * L) / (2.0 * R)
        v_right_rad = (2.0 * v + w * L) / (2.0 * R)

        # rad/s -> STS3215 step/s 변환 (1 회전 = 4096 steps)
        RAD_TO_STEP = 4096.0 / (2.0 * 3.1415926535)

        dir_left = -1.0 if self.invert_left else 1.0
        dir_right = -1.0 if self.invert_right else 1.0

        speed_left = dir_left * self.scale_left * v_left_rad * RAD_TO_STEP
        speed_right = dir_right * self.scale_right * v_right_rad * RAD_TO_STEP

        # 모든 바퀴에 대해 Sync Write로 단일 패킷 동시 전송 (타이밍 오차 제로화)
        speeds = {
            self.left_id: speed_left,
            self.right_id: speed_right
        }
        if self.four_wheel_mode:
            speeds[self.left_rear_id] = speed_left
            speeds[self.right_rear_id] = speed_right

        self.send_sync_speeds(speeds)

    def left_wheel_callback(self, msg: Float32):
        self.send_speed(self.left_id, msg.data)
        if self.four_wheel_mode:
            self.send_speed(self.left_rear_id, msg.data)

    def right_wheel_callback(self, msg: Float32):
        self.send_speed(self.right_id, msg.data)
        if self.four_wheel_mode:
            self.send_speed(self.right_rear_id, msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StsControllerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            if hasattr(node, 'serial_port') and node.serial_port is not None and node.serial_port.is_open:
                node.serial_port.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()