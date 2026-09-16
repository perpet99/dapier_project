// --- 하드웨어 구성 ---
// - 모터 드라이버: L298N x1 (2채널, ENA/ENB는 사용하지 않고 IN1~IN4 방향핀만으로 on/off 제어)
// - 모터: 2개 (왼쪽/오른쪽)
// - 전원: 모터 구동 전원(18650 배터리)과 아두이노 로직 전원은 분리하여 공급 (GND는 공통)
// - 모터 제어는 별도 Motor 클래스(Motor.h / Motor.cpp)로 구현, 왼쪽/오른쪽 2개 객체 생성

#include "Motor.h"
#include <SoftwareSerial.h>

// 왼쪽 모터: L298N 채널A (IN1=5, IN2=6)
const uint8_t MOTOR_L_DIR1 = 5;
const uint8_t MOTOR_L_DIR2 = 6;

// 오른쪽 모터: L298N 채널B (IN3=10, IN4=11)
const uint8_t MOTOR_R_DIR1 = 10;
const uint8_t MOTOR_R_DIR2 = 11;

Motor motorL(MOTOR_L_DIR1, MOTOR_L_DIR2);
Motor motorR(MOTOR_R_DIR1, MOTOR_R_DIR2);

// ESP32 WiFi 브리지 수신 전용 (하드웨어 Serial은 USB/PC 디버그 전용으로 분리)
// 배선: ESP32 TX2(GPIO17) -> Arduino D7, GND 공통 (D4는 미사용, 배선 안 함)
SoftwareSerial espSerial(7, 8); // RX=D7, TX=D4(미사용)

void setup() {
  Serial.begin(115200);
  espSerial.begin(9600);   // SoftwareSerial은 고속에서 불안정하므로 저속 고정 (esp32.ino의 Serial2와 동일하게)
  espSerial.setTimeout(50);

  motorL.begin();
  motorR.begin();

  Serial.println("READY");
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() > 0) {
      Serial.print("[RX USB] ");
      Serial.println(line);
      handleCommand(line, "USB");
    }
  }

  if (espSerial.available() > 0) {
    String line = espSerial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      Serial.print("[RX WiFi] ");
      Serial.println(line);
      handleCommand(line, "WiFi");
    }
  }
}

// 파이썬 UI(USB)와 ESP32 WiFi 브리지(esp32.ino) 공통 통신 프로토콜
// (PWM 없이 on/off 제어이므로 값의 부호만 사용)
//   "L,<speed>"  : 왼쪽 모터 방향 설정 (양수=정방향, 음수=역방향, 0=정지)
//   "R,<speed>"  : 오른쪽 모터 방향 설정 (양수=정방향, 음수=역방향, 0=정지)
//   "STOP"       : 두 모터 모두 정지
void handleCommand(const String &line, const char *source) {
  if (line == "STOP") {
    motorL.stop();
    motorR.stop();
    Serial.print("ACK,STOP,");
    Serial.println(source);
    return;
  }

  int commaIndex = line.indexOf(',');
  if (commaIndex <= 0) return;

  String target = line.substring(0, commaIndex);
  int speed = line.substring(commaIndex + 1).toInt();

  if (target == "L") {
    motorL.setSpeed(speed);
    Serial.print("ACK,L,");
    Serial.print(speed);
    Serial.print(",");
    Serial.println(source);
  } else if (target == "R") {
    motorR.setSpeed(speed);
    Serial.print("ACK,R,");
    Serial.print(speed);
    Serial.print(",");
    Serial.println(source);
  }
}
