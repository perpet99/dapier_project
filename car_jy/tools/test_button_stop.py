#!/usr/bin/env python3
"""
AMR Raspberry Pi Drive & Button Stop Test Tool
로봇을 직진 주행시키다가 물리 버튼이 눌리면 즉시 비상 정지하는 테스트 프로그램

- 동작 과정:
  1) 3초 카운트다운 후 로봇이 안전 속도(기본 0.08 m/s)로 직진 주행을 시작합니다.
  2) 로봇이 전진하는 동안 라즈베리파이 GPIO 버튼 입력을 실시간 감시합니다.
  3) 버튼이 딸깍 눌리는 순간 즉시 모터를 0.0 m/s로 멈추고 주행을 중단합니다.
  4) 안전 가드: 버튼을 누르지 않더라도 최대 주행 시간(기본 15초) 초과 시 벽 충돌 방지를 위해 자동 정지합니다.
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


class DriveAndStopController:
    """로봇 직진 주행 및 비상 정지 제어기"""
    def __init__(self, topic_name='/cmd_vel', web_port=8088, speed=0.08, max_duration=15.0):
        self.topic_name = topic_name
        self.web_port = web_port
        self.speed = speed
        self.max_duration = max_duration

        self.node = None
        self.publisher = None
        self.executor = None
        self.spin_thread = None

        self.is_running = True
        self.is_stopped = False
        self.stopped_by_button = False
        self.stop_lock = threading.Lock()

        if HAS_ROS2:
            try:
                if not rclpy.ok():
                    rclpy.init()
                self.node = rclpy.create_node('button_drive_stop_test')
                self.publisher = self.node.create_publisher(Twist, self.topic_name, 10)
                
                # 백그라운드 ROS 2 spin
                self.executor = rclpy.executors.SingleThreadedExecutor()
                self.executor.add_node(self.node)

                def _safe_spin():
                    try:
                        self.executor.spin()
                    except (rclpy.executors.ExternalShutdownException, Exception):
                        pass

                self.spin_thread = threading.Thread(target=_safe_spin, daemon=True)
                self.spin_thread.start()
                print(f"🤖 [ROS 2] 노드 초기화 완료 (속도 명령 토픽: {self.topic_name})")
            except Exception as e:
                print(f"⚠️ [ROS 2] 노드 생성 실패: {e}")
        else:
            print("⚠️ [ROS 2] rclpy 라이브러리가 없어 모터 명령을 발행할 수 없습니다.")

    def trigger_button_stop(self):
        """버튼 감지 시 즉시 호출되는 정지 함수"""
        with self.stop_lock:
            if self.is_stopped:
                return
            self.is_stopped = True
            self.stopped_by_button = True

        print("\n" + "="*60)
        print("🛑 [EMERGENCY STOP] 🔘 물리 버튼이 눌렸습니다!")
        print("   로봇 주행을 즉각 차단하고 모터를 완전히 정지합니다!")
        print("="*60)

        # 1. 속도 0 명령 강력 발행 (15회 연속)
        if self.publisher is not None:
            stop_twist = Twist()
            for _ in range(15):
                self.publisher.publish(stop_twist)
                time.sleep(0.015)
            print(f"  ✓ {self.topic_name} 에 속도 0 (Twist: 0.0, 0.0) 15회 발행 완료")

        # 2. 실행 중인 정밀 도킹 노드 /api/stop 호출
        try:
            url = f"http://127.0.0.1:{self.web_port}/api/stop"
            req = urllib.request.Request(url, headers={'User-Agent': 'AMR-Button-Daemon'})
            with urllib.request.urlopen(req, timeout=0.8) as resp:
                data = resp.read().decode('utf-8')
                print(f"  ✓ 도킹 노드 정지 신호 전송 완료 (/api/stop 응답: {data.strip()})")
        except Exception:
            pass

        print("="*60)
        print("🎉 [테스트 성공] 버튼 정지 동작이 정상적으로 확인되었습니다!\n")

    def run_drive_test(self):
        """카운트다운 후 로봇 직진 주행 루프 실행"""
        print("\n⚠️ [주의] 로봇 주변 1~2m 반경에 장애물이 없는지 확인하세요.")
        for i in range(3, 0, -1):
            print(f"⏳ {i}초 후 로봇이 직진을 시작합니다... (버튼 누를 준비를 하세요!)")
            time.sleep(1.0)

        print("\n🚀 [직진 시작!] 로봇이 전진 중입니다. 🔘 버튼을 누르면 멈춥니다!")
        start_time = time.time()

        drive_twist = Twist()
        drive_twist.linear.x = float(self.speed)
        drive_twist.angular.z = 0.0

        try:
            while self.is_running and not self.is_stopped:
                now = time.time()
                elapsed = now - start_time
                remaining = max(0.0, self.max_duration - elapsed)

                if elapsed >= self.max_duration:
                    print(f"\n⏰ [안전 타임아웃] {self.max_duration:.1f}초 동안 버튼 입력이 없어 안전을 위해 자동 정지합니다.")
                    self.stop_robot_safety()
                    break

                # 전진 명령 발행 (10Hz)
                if self.publisher is not None and not self.is_stopped:
                    self.publisher.publish(drive_twist)

                # 한 줄 실시간 상태 표시
                sys.stdout.write(f"\r🚗 [전진 중] 속도: {self.speed:+.2f} m/s | ⏱️ 경과: {elapsed:.1f}s / 남은 시간: {remaining:.1f}s  👉 [버튼을 누르세요!] ")
                sys.stdout.flush()

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n\n⏹️ [사용자 중단] Ctrl+C 로 주행을 멈춥니다.")
            self.stop_robot_safety()

    def stop_robot_safety(self):
        """안전 목적 또는 종료 시 정지"""
        with self.stop_lock:
            self.is_stopped = True
        if self.publisher is not None:
            stop_twist = Twist()
            for _ in range(10):
                self.publisher.publish(stop_twist)
                time.sleep(0.02)
        print("  ✓ 로봇 모터 정지 완료")

    def shutdown(self):
        self.is_running = False
        self.stop_robot_safety()
        if self.executor is not None:
            try:
                self.executor.shutdown()
            except Exception:
                pass
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
        if HAS_ROS2 and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


def setup_gpio(pin_num, controller, debounce_sec=0.05):
    """
    라즈베리파이 GPIO 이벤트 등록
    gpiozero 우선 -> RPi.GPIO -> gpiod 순으로 감지
    """
    # 1. gpiozero 시도
    try:
        from gpiozero import Button
        print(f"🔌 [GPIO] gpiozero 라이브러리 활성화 (BCM 핀: {pin_num}, 내부 풀업 ON)")
        btn = Button(pin_num, pull_up=True, bounce_time=debounce_sec)
        btn.when_pressed = controller.trigger_button_stop
        btn.when_released = lambda: print("\n⚪ [BUTTON] 버튼에서 손을 뗐습니다.")

        # 🌟 초고속 폴링(100Hz) 백업 스레드 (인터럽트 누락 원천 차단)
        def poll_gpiozero():
            while controller.is_running and not controller.is_stopped:
                try:
                    if btn.is_pressed:
                        controller.trigger_button_stop()
                        break
                except Exception:
                    pass
                time.sleep(0.01)
        threading.Thread(target=poll_gpiozero, daemon=True).start()
        return "gpiozero", btn
    except ImportError:
        pass

    # 2. RPi.GPIO 시도
    try:
        import RPi.GPIO as GPIO
        print(f"🔌 [GPIO] RPi.GPIO 라이브러리 활성화 (BCM 핀: {pin_num}, 내부 풀업 ON)")
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin_num, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        def bcm_callback(channel):
            controller.trigger_button_stop()

        GPIO.add_event_detect(pin_num, GPIO.FALLING, callback=bcm_callback, bouncetime=int(debounce_sec * 1000))

        # 🌟 초고속 폴링(100Hz) 백업 스레드
        def poll_rpi():
            while controller.is_running and not controller.is_stopped:
                try:
                    if GPIO.input(pin_num) == 0:
                        controller.trigger_button_stop()
                        break
                except Exception:
                    pass
                time.sleep(0.01)
        threading.Thread(target=poll_rpi, daemon=True).start()
        return "RPi.GPIO", GPIO
    except ImportError:
        pass

    # 3. gpiod 시도 (라즈베리파이 OS 기본 내장)
    try:
        import gpiod
        chip_name = 'gpiochip4' if os.path.exists('/dev/gpiochip4') else 'gpiochip0'
        chip = gpiod.Chip(chip_name)
        line = chip.get_line(pin_num)
        req_flags = getattr(gpiod, 'LINE_REQ_FLAG_BIAS_PULL_UP', 0)
        line.request(consumer="amr_button", type=gpiod.LINE_REQ_DIR_IN, flags=req_flags)

        def poll_gpiod():
            while controller.is_running and not controller.is_stopped:
                try:
                    if line.get_value() == 0:
                        controller.trigger_button_stop()
                        break
                except Exception:
                    pass
                time.sleep(0.01)
        threading.Thread(target=poll_gpiod, daemon=True).start()
        print(f"🔌 [GPIO] gpiod 라이브러리 활성화 ({chip_name}, BCM 핀: {pin_num})")
        return "gpiod", line
    except Exception:
        pass

    return None, None


def main():
    parser = argparse.ArgumentParser(description="로봇 직진 주행 중 버튼 정지 테스트")
    parser.add_argument('--pin', type=int, default=18, help="버튼 연결 BCM GPIO 핀 번호 (기본: 18 = 물리 12번 핀)")
    parser.add_argument('--speed', type=float, default=0.08, help="직진 테스트 주행 속도 (m/s, 기본: 0.08)")
    parser.add_argument('--topic', type=str, default='/cmd_vel', help="주행 제어 토픽 (기본: /cmd_vel)")
    parser.add_argument('--duration', type=float, default=15.0, help="최대 직진 주행 허용 시간 (초, 기본: 15.0)")
    parser.add_argument('--port', type=int, default=8088, help="도킹 노드 웹 API 포트 (기본: 8088)")
    args = parser.parse_args()

    print("\n" + "="*60)
    print("   🏷️ AMR 로봇 직진 주행 및 물리 버튼 비상 정지 테스트")
    print("="*60)
    print(f" • 연결 GPIO 핀     : BCM {args.pin} (물리 핀 번호 12번)")
    print(f" • 접지(GND) 핀     : 물리 핀 번호 14번")
    print(f" • 테스트 직진 속도 : {args.speed:.2f} m/s")
    print(f" • 최대 주행 시간   : {args.duration:.1f} 초 (미입력 시 자동 정지)")
    print(f" • 속도 제어 토픽   : {args.topic}")
    print("="*60)

    controller = DriveAndStopController(
        topic_name=args.topic,
        web_port=args.port,
        speed=args.speed,
        max_duration=args.duration
    )

    backend, gpio_handle = setup_gpio(args.pin, controller)

    if backend is None:
        print("\n⚠️ [안내] 라즈베리파이 GPIO 라이브러리가 감지되지 않았습니다.")
        print("  다음 중 하나로 설치할 수 있습니다:")
        print("    1) sudo rm -f /etc/apt/sources.list.d/*antigravity* && sudo apt update && sudo apt install -y python3-gpiozero")
        print("    2) pip install gpiozero --break-system-packages")
        print("\n💡 [키보드 시뮬레이션 모드]")
        print("  엔터(Enter) 키를 누르면 직진을 시작하고, 다시 엔터를 누르면 비상 정지합니다.")
        input("👉 준비가 완료되면 Enter 키를 누르세요...")

        # 시뮬레이션 키보드 감시 스레드
        def keyboard_listener():
            input("\n👉 [스페이스/엔터] 지금 정지하려면 Enter를 누르세요!\n")
            controller.trigger_button_stop()

        t_kb = threading.Thread(target=keyboard_listener, daemon=True)
        t_kb.start()
    else:
        print(f"✅ GPIO 버튼 감지기가 준비되었습니다! (백엔드: {backend})\n")

    try:
        # 로봇 직진 및 버튼 정지 실행
        controller.run_drive_test()
    finally:
        if backend == "RPi.GPIO":
            gpio_handle.cleanup()
        controller.shutdown()
        print("\n프로그램이 안전하게 종료되었습니다.")


if __name__ == '__main__':
    main()
