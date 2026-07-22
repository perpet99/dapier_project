"""
exam2.ino(L298N 모터 2개)를 시리얼로 제어하는 파이썬 UI.

필요 패키지: pyserial (tkinter는 표준 라이브러리)
    pip install pyserial

사용법:
    python motor_ui.py
    -> UI에서 포트를 선택/입력하고 Connect, 이후 전진/후진/좌/우 버튼을 누르고 있는 동안 이동.
       버튼에서 손을 떼면 자동 정지.

시리얼 프로토콜 (exam2.ino와 동일, PWM 없이 on/off 제어이므로 부호만 사용):
    "L,<value>\n"  : 왼쪽 모터 방향 설정 (양수=정방향, 음수=역방향, 0=정지)
    "R,<value>\n"  : 오른쪽 모터 방향 설정 (양수=정방향, 음수=역방향, 0=정지)
    "STOP\n"       : 두 모터 모두 정지
"""

import threading
import tkinter as tk
from tkinter import messagebox, ttk

import serial
import serial.tools.list_ports


class MotorSerial:
    """모터 제어 명령을 아두이노로 보내는 시리얼 연결 래퍼."""

    def __init__(self):
        self.ser = None
        self.lock = threading.Lock()

    def connect(self, port, baudrate=115200):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(port, baudrate, timeout=0.1)

    def disconnect(self):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = None

    def is_connected(self):
        with self.lock:
            return self.ser is not None and self.ser.is_open

    def send(self, cmd):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.write((cmd + "\n").encode("utf-8"))

    def read_all_lines(self):
        lines = []
        with self.lock:
            if self.ser and self.ser.is_open:
                while self.ser.in_waiting > 0:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        lines.append(line)
        return lines


class MotorControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("L298N Motor Control")

        self.motor_serial = MotorSerial()

        self._build_ui()
        self._poll_serial()

    def _build_ui(self):
        conn_frame = ttk.LabelFrame(self.root, text="연결")
        conn_frame.pack(fill="x", padx=10, pady=5)

        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15)
        self.port_combo["values"] = self._list_ports()
        if self.port_combo["values"]:
            self.port_combo.current(0)
        self.port_combo.pack(side="left", padx=5, pady=5)

        ttk.Button(conn_frame, text="새로고침", command=self._refresh_ports).pack(side="left", padx=5)
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self._toggle_connect)
        self.connect_btn.pack(side="left", padx=5)

        self.status_label = ttk.Label(conn_frame, text="연결 안됨", foreground="red")
        self.status_label.pack(side="left", padx=10)

        control_frame = ttk.LabelFrame(self.root, text="모터 제어 (누르고 있는 동안 이동)")
        control_frame.pack(fill="x", padx=10, pady=10)

        pad_frame = ttk.Frame(control_frame)
        pad_frame.pack(pady=10)

        btn_forward = ttk.Button(pad_frame, text="▲\n전진")
        btn_backward = ttk.Button(pad_frame, text="▼\n후진")
        btn_left = ttk.Button(pad_frame, text="◀\n좌")
        btn_right = ttk.Button(pad_frame, text="▶\n우")
        btn_stop = ttk.Button(pad_frame, text="■\n정지", command=self._stop_all)

        btn_forward.grid(row=0, column=1, ipadx=10, ipady=10, padx=3, pady=3)
        btn_left.grid(row=1, column=0, ipadx=10, ipady=10, padx=3, pady=3)
        btn_stop.grid(row=1, column=1, ipadx=10, ipady=10, padx=3, pady=3)
        btn_right.grid(row=1, column=2, ipadx=10, ipady=10, padx=3, pady=3)
        btn_backward.grid(row=2, column=1, ipadx=10, ipady=10, padx=3, pady=3)

        self._bind_hold(btn_forward, self._forward)
        self._bind_hold(btn_backward, self._backward)
        self._bind_hold(btn_left, self._turn_left)
        self._bind_hold(btn_right, self._turn_right)

        log_frame = ttk.LabelFrame(self.root, text="아두이노 로그")
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.log_text = tk.Text(log_frame, height=8, state="disabled")
        self.log_text.pack(fill="both", expand=True, padx=5, pady=5)

    def _bind_hold(self, button, action):
        # 버튼을 누르고 있는 동안 이동하고, 손을 떼면 정지 (RC 조종 방식)
        button.bind("<ButtonPress-1>", lambda e: action())
        button.bind("<ButtonRelease-1>", lambda e: self._stop_all())

    def _forward(self):
        self.motor_serial.send("L,1")
        self.motor_serial.send("R,1")

    def _backward(self):
        self.motor_serial.send("L,-1")
        self.motor_serial.send("R,-1")

    def _turn_left(self):
        self.motor_serial.send("L,-1")
        self.motor_serial.send("R,1")

    def _turn_right(self):
        self.motor_serial.send("L,1")
        self.motor_serial.send("R,-1")

    def _stop_all(self):
        self.motor_serial.send("STOP")

    def _list_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def _refresh_ports(self):
        self.port_combo["values"] = self._list_ports()

    def _toggle_connect(self):
        if self.motor_serial.is_connected():
            self.motor_serial.disconnect()
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="연결 안됨", foreground="red")
            return

        port = self.port_var.get()
        if not port:
            messagebox.showwarning("포트 선택", "먼저 포트를 선택하세요.")
            return

        try:
            self.motor_serial.connect(port)
        except serial.SerialException as e:
            messagebox.showerror("연결 실패", str(e))
            return

        self.connect_btn.config(text="Disconnect")
        self.status_label.config(text=f"연결됨 ({port})", foreground="green")

    def _poll_serial(self):
        if self.motor_serial.is_connected():
            for line in self.motor_serial.read_all_lines():
                self.log_text.config(state="normal")
                self.log_text.insert("end", line + "\n")
                self.log_text.see("end")
                self.log_text.config(state="disabled")

        self.root.after(100, self._poll_serial)

    def on_close(self):
        self.motor_serial.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = MotorControlApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
