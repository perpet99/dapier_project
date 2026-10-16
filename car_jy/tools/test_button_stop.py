#!/usr/bin/env python3
"""
AMR Raspberry Pi Button Test & Emergency Stop Daemon
라즈베리파이 물리 버튼 테스트 및 로봇 비상 정지(Emergency Stop) 스크립트

- 버튼이 눌리면:
  1) ROS 2 /cmd_vel 토픽으로 0.0 m/s 속도 명령(정지) 연속 발행
  2) 실행 중인 정밀 도킹 노드(포트 8088)에 /api/stop 요청 전송하여 도킹 즉시 중단
  3) 콘솔에 정지 상태 및 경고 출력
"""

import os
import sys
import time
import argparse
import threading
import urllib.request
import json

# ROS 2 라이브러리 (선택적 임포트)
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


class EmergencyStopNode:
    """ROS 2 cmd_vel 퍼블리셔 및 정지 제어기"""
    def __init__(self, topic_name='/cmd_vel', web_port=8088):
        self.topic_name = topic_name
        self.web_port = web_port
        self.node = None
        self.publisher = None
        self.executor = None
        self.spin_thread = None

        if HAS_ROS2:
            try:
                if not rclpy.ok():
                    rclpy.init()
                self.node = rclpy.create_node('button_emergency_stop_node')
                self.publisher = self.node.create_publisher(Twist, self.topic_name, 10)
                
                # 백그라운드 spin
                self.executor = rclpy.executors.SingleThreadedExecutor()
                self.executor.add_node(self.node)
                self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
                self.spin_thread.start()
                print(f"🤖 [ROS 2] 노드 초기화 완료 (정지 토픽: {self.topic_name})")
            except Exception as e:
                print(f"⚠️ [ROS 2] 노드 생성 실패: {e}")
        else:
            print("⚠️ [ROS 2] rclpy를 찾을 수 없습니다. (HTTP API 정지만 동작합니다)")

    def stop_robot(self):
        """로봇 즉시 정지 실행"""
        print("\n" + "="*56)
        print("🛑 [EMERGENCY STOP] 버튼 감지! 로봇을 즉시 정지합니다!")
        print("="*56)

        # 1. ROS 2 속도 0 연속 발행 (10회, 확실한 모터 정지 보장)
        if self.publisher is not None:
            stop_twist = Twist()
            stop_twist.linear.x = 0.0
            stop_twist.linear.y = 0.0
            stop_twist.linear.z = 0.0
            stop_twist.angular.x = 0.0
            stop_twist.angular.y = 0.0
            stop_twist.angular.z = 0.0

            for _ in range(10):
                self.publisher.publish(stop_twist)
                time.sleep(0.02)
            print(f"  ✓ {self.topic_name} 에 속도 0 (Twist: 0.0, 0.0) 발행 완료")

        # 2. 실행 중인 도킹 노드 웹 API 정지 호출
        try:
            url = f"http://127.0.0.1:{self.web_port}/api/stop"
            req = urllib.request.Request(url, headers={'User-Agent': 'AMR-Button-Daemon'})
            with urllib.request.urlopen(req, timeout=0.8) as resp:
                data = resp.read().decode('utf-8')
                print(f"  ✓ 정밀 도킹 노드 정지 신호 전송 완료 (/api/stop 응답: {data.strip()})")
        except Exception:
            # 도킹 노드가 켜져있지 않은 일반 주행 상태일 수 있음
            pass

        print("="*56 + "\n")

    def shutdown(self):
        if self.node is not None:
            self.node.destroy_node()
        if HAS_ROS2 and rclpy.ok():
            rclpy.shutdown()


def setup_gpio(pin_num, stop_controller, debounce_sec=0.05):
    """
    라즈베리파이 GPIO 이벤트 등록
    gpiozero 우선 시도 -> RPi.GPIO 차선 시도 -> 없을 시 안내 메시지
    """
    # 1. gpiozero 시도 (최신 라즈베리파이 OS 기본 권장)
    try:
        from gpiozero import Button
        print(f"🔌 [GPIO] gpiozero 라이브러리 사용 (BCM 핀: {pin_num}, 내부 풀업 ON)")
        btn = Button(pin_num, pull_up=True, bounce_time=debounce_sec)

        btn.when_pressed = stop_controller.stop_robot
        btn.when_released = lambda: print("⚪ [BUTTON] 버튼에서 손을 뗐습니다. (정지 상태 유지)")
        return "gpiozero", btn
    except ImportError:
        pass

    # 2. RPi.GPIO 시도
    try:
        import RPi.GPIO as GPIO
        print(f"🔌 [GPIO] RPi.GPIO 라이브러리 사용 (BCM 핀: {pin_num}, 내부 풀업 ON)")
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin_num, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        def bcm_callback(channel):
            stop_controller.stop_robot()

        GPIO.add_event_detect(pin_num, GPIO.FALLING, callback=bcm_callback, bouncetime=int(debounce_sec*1000))
        return "RPi.GPIO", GPIO
    except ImportError:
        pass

    # 3. gpiod 시도 (라즈베리파이 OS 기본 내장 라이브러리)
    try:
        import gpiod
        chip_name = 'gpiochip4' if os.path.exists('/dev/gpiochip4') else 'gpiochip0'
        chip = gpiod.Chip(chip_name)
        line = chip.get_line(pin_num)
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

        t = GpiodThread(line, stop_controller.stop_robot)
        t.start()
        print(f"🔌 [GPIO] gpiod 라이브러리 사용 ({chip_name}, BCM 핀: {pin_num})")
        return "gpiod", line
    except Exception:
        pass

    return None, None


def main():
    parser = argparse.ArgumentParser(description="라즈베리파이 물리 버튼 테스트 및 로봇 정지 데몬")
    parser.add_argument('--pin', type=int, default=18, help="버튼이 연결된 BCM GPIO 핀 번호 (기본: 18 = 물리 12번 핀)")
    parser.add_argument('--topic', type=str, default='/cmd_vel', help="정지 속도 명령을 보낼 토픽 (기본: /cmd_vel)")
    parser.add_argument('--port', type=int, default=8088, help="도킹 노드 웹 API 포트 (기본: 8088)")
    parser.add_argument('--bounce', type=float, default=0.05, help="버튼 바운스 방지 시간 (초, 기본: 0.05)")
    args = parser.parse_args()

    print("\n" + "="*56)
    print("   🏷️ AMR 라즈베리파이 버튼 테스트 & 비상 정지 프로그램")
    print("="*56)
    print(f" • 연결 GPIO 핀 : BCM {args.pin} (물리 핀 번호 12번)")
    print(f" • 접지 (GND) 핀: 물리 핀 번호 14번 권장")
    print(f" • 정지 명령 토픽: {args.topic}")
    print(f" • 도킹 정지 포트: {args.port}")
    print("="*56 + "\n")

    stop_controller = EmergencyStopNode(topic_name=args.topic, web_port=args.port)
    backend, gpio_handle = setup_gpio(args.pin, stop_controller, debounce_sec=args.bounce)

    if backend is None:
        print("⚠️ [안내] 라즈베리파이 GPIO 라이브러리가 감지되지 않았습니다.")
        print("  라즈베리파이 터미널에서 다음 명령어로 설치할 수 있습니다:")
        print("    sudo apt update && sudo apt install -y python3-gpiozero")
        print("\n💡 [테스트 모드 활성화]")
        print("  키보드 [엔터] 키를 누르면 비상 정지가 트리거됩니다. (종료: Ctrl+C)")
        try:
            while True:
                input("👉 [테스트] 버튼을 누른 것처럼 시뮬레이션하려면 Enter를 누르세요...\n")
                stop_controller.stop_robot()
        except KeyboardInterrupt:
            print("\n프로그램을 종료합니다.")
        finally:
            stop_controller.shutdown()
        return

    print("🚀 [준비 완료] 버튼을 누르면 로봇이 즉시 정지합니다!")
    print("   종료하려면 Ctrl + C 를 누르세요.\n")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n프로그램 종료 중...")
    finally:
        if backend == "RPi.GPIO":
            gpio_handle.cleanup()
        stop_controller.shutdown()
        print("정상적으로 종료되었습니다.")


if __name__ == '__main__':
    main()
