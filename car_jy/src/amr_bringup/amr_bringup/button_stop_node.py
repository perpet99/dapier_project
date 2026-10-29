#!/usr/bin/env python3
"""
ROS 2 Button Stop Node
라즈베리파이 물리 버튼을 감지하여 로봇 비상 정지 명령을 발행하는 ROS 2 노드
"""

import sys
import time
import urllib.request
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class ButtonStopNode(Node):
    def __init__(self):
        super().__init__('button_stop_node')

        self.declare_parameter('pin', 18)                      # BCM GPIO 핀 번호 (기본: 18)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')     # 정지 속도 토픽 (기본: /cmd_vel)
        self.declare_parameter('web_port', 8088)                # 도킹 웹 API 포트 (기본: 8088)
        self.declare_parameter('bounce_time', 0.05)             # 채터링 방지 시간 (초)

        self.pin = int(self.get_parameter('pin').value)
        self.topic_name = str(self.get_parameter('cmd_vel_topic').value)
        self.web_port = int(self.get_parameter('web_port').value)
        self.bounce_time = float(self.get_parameter('bounce_time').value)

        self.cmd_pub = self.create_publisher(Twist, self.topic_name, 10)
        self.pub_bumper = self.create_publisher(Bool, '/bumper_state', 10)
        self.get_logger().info(f"🔘 [ButtonStopNode] 초기화 완료 (GPIO BCM {self.pin}, 토픽: {self.topic_name}, 범퍼: /bumper_state)")

        self.gpio_backend = None
        self.btn_handle = None
        self._init_gpio()

    def _init_gpio(self):
        try:
            from gpiozero import Button
            self.btn_handle = Button(self.pin, pull_up=True, bounce_time=self.bounce_time)
            self.btn_handle.when_pressed = self.on_button_pressed
            self.btn_handle.when_released = self.on_button_released
            self.gpio_backend = "gpiozero"
            self.get_logger().info(f"🔌 [GPIO] gpiozero 활성화 (BCM {self.pin}번 핀 대기 중)")
            return
        except ImportError:
            pass

        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(
                self.pin, GPIO.FALLING, callback=lambda ch: self.on_button_pressed(),
                bouncetime=int(self.bounce_time * 1000)
            )
            self.gpio_backend = "RPi.GPIO"
            self.get_logger().info(f"🔌 [GPIO] RPi.GPIO 활성화 (BCM {self.pin}번 핀 대기 중)")
            return
        except ImportError:
            pass

        try:
            import os
            import threading
            import gpiod
            chip_name = 'gpiochip4' if os.path.exists('/dev/gpiochip4') else 'gpiochip0'
            chip = gpiod.Chip(chip_name)
            line = chip.get_line(self.pin)
            req_flags = getattr(gpiod, 'LINE_REQ_FLAG_BIAS_PULL_UP', 0)
            line.request(consumer="amr_button", type=gpiod.LINE_REQ_EV_FALLING_EDGE, flags=req_flags)

            class GpiodThread(threading.Thread):
                def __init__(self, line, callback):
                    super().__init__(daemon=True)
                    self.line = line
                    self.callback = callback
                    self.running = True
                def run(self):
                    while self.running:
                        try:
                            if self.line.event_wait(sec=1):
                                _ = self.line.event_read()
                                self.callback()
                        except Exception:
                            time.sleep(0.1)

            self.gpiod_thread = GpiodThread(line, self.on_button_pressed)
            self.gpiod_thread.start()
            self.gpio_backend = "gpiod"
            self.get_logger().info(f"🔌 [GPIO] gpiod 활성화 ({chip_name}, BCM {self.pin}번 핀 대기 중)")
            return
        except Exception:
            pass

        self.get_logger().warn(
            "⚠️ [GPIO] gpiozero, RPi.GPIO 또는 gpiod 라이브러리를 찾을 수 없습니다. "
            "(라즈베리파이에서 'sudo apt install python3-gpiozero' 실행 필요)"
        )

    def on_button_pressed(self):
        self.get_logger().warn("🛑 [EMERGENCY STOP & BUMPER] 물리 버튼 감지! 로봇 즉시 정지 및 범퍼 접촉 신호 발행!")

        # 1. 범퍼 접촉 신호(/bumper_state) 발행 -> 도킹 노드에서 DOCKED 상태 전환 트리거
        bumper_msg = Bool()
        bumper_msg.data = True
        self.pub_bumper.publish(bumper_msg)

        # 2. ROS 2 속도 0 연속 10회 발행 (모터 즉각 차단)
        stop_msg = Twist()
        for _ in range(10):
            self.cmd_pub.publish(stop_msg)
            time.sleep(0.02)

        # 3. 실행 중인 정밀 도킹 노드 /api/stop 호출
        try:
            url = f"http://127.0.0.1:{self.web_port}/api/stop"
            req = urllib.request.Request(url, headers={'User-Agent': 'AMR-Button-Node'})
            with urllib.request.urlopen(req, timeout=0.8) as resp:
                data = resp.read().decode('utf-8')
                self.get_logger().info(f"  ✓ 도킹 정지 신호 완료 (/api/stop 응답: {data.strip()})")
        except Exception:
            pass

    def on_button_released(self):
        bumper_msg = Bool()
        bumper_msg.data = False
        self.pub_bumper.publish(bumper_msg)
        self.get_logger().info("⚪ [BUTTON] 버튼 해제됨 (정지 상태 유지)")


def main(args=None):
    rclpy.init(args=args)
    node = ButtonStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
