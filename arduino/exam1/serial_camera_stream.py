"""
OpenCV 카메라 라이브 스트림 + 아두이노 시리얼 통신을 동시에 수행하는 스크립트.

- 메인 스레드: while 루프에서 OpenCV로 카메라를 띄우고 FPS를 오버레이 표시.
- 별도 스레드(SerialWorker): 아두이노와 시리얼로 송수신 (연결된 exam1.ino와 짝을 이룸).

필요 패키지: opencv-python, pyserial
    pip install opencv-python pyserial

사용법:
    python serial_camera_stream.py --port COM3 --baud 115200 --camera 0

카메라 창에서:
    o : 아두이노에 "LED_ON" 명령 전송
    f : 아두이노에 "LED_OFF" 명령 전송
    q : 종료
"""

import argparse
import queue
import threading
import time

import cv2
import serial


class SerialWorker(threading.Thread):
    """아두이노와의 시리얼 송수신을 전담하는 스레드."""

    def __init__(self, port, baudrate, command_queue, stop_event):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.command_queue = command_queue
        self.stop_event = stop_event

        self.lock = threading.Lock()
        self.latest_status = ""
        self.connected = False

    def get_latest_status(self):
        with self.lock:
            return self.latest_status

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        except serial.SerialException as e:
            with self.lock:
                self.latest_status = f"[Serial Error] {e}"
            return

        time.sleep(2)  # 아두이노 리셋 및 부팅 대기
        self.connected = True

        while not self.stop_event.is_set():
            # 1. 파이썬 -> 아두이노: 큐에 쌓인 명령 전송
            try:
                cmd = self.command_queue.get_nowait()
                ser.write((cmd + "\n").encode("utf-8"))
            except queue.Empty:
                pass

            # 2. 아두이노 -> 파이썬: 수신 데이터 읽기
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        with self.lock:
                            self.latest_status = line
            except serial.SerialException as e:
                with self.lock:
                    self.latest_status = f"[Serial Error] {e}"
                break

        ser.close()
        self.connected = False

    def send_command(self, cmd):
        self.command_queue.put(cmd)


def main():
    parser = argparse.ArgumentParser(description="OpenCV + Arduino Serial (threaded)")
    parser.add_argument("--port", default="COM3", help="아두이노 시리얼 포트 (예: COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="시리얼 통신 속도")
    parser.add_argument("--camera", type=int, default=0, help="카메라 장치 인덱스")
    args = parser.parse_args()

    stop_event = threading.Event()
    command_queue = queue.Queue()

    serial_worker = SerialWorker(args.port, args.baud, command_queue, stop_event)
    serial_worker.start()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        stop_event.set()
        serial_worker.join()
        return

    prev_time = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("프레임을 읽을 수 없습니다.")
                break

            # FPS 계산
            now = time.time()
            fps = 1.0 / (now - prev_time) if now > prev_time else 0.0
            prev_time = now

            cv2.putText(
                frame, f"FPS: {fps:.1f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2,
            )
            cv2.putText(
                frame, f"Arduino: {serial_worker.get_latest_status()}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2,
            )

            cv2.imshow("Camera Stream", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('o'):
                serial_worker.send_command("LED_ON")
            elif key == ord('f'):
                serial_worker.send_command("LED_OFF")
    finally:
        stop_event.set()
        serial_worker.join(timeout=2)
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
