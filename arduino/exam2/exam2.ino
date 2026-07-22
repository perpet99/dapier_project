// --- 하드웨어 구성 ---
// - 모터 드라이버: L298N x1 (2채널, ENA/ENB는 사용하지 않고 IN1~IN4 방향핀만으로 on/off 제어)
// - 모터: 2개 (왼쪽/오른쪽)
// - 전원: 모터 구동 전원(18650 배터리)과 아두이노 로직 전원은 분리하여 공급 (GND는 공통)
// - 모터 제어는 별도 Motor 클래스(Motor.h / Motor.cpp)로 구현, 왼쪽/오른쪽 2개 객체 생성

#include "Motor.h"

// 왼쪽 모터: L298N 채널A (IN1=5, IN2=6)
const uint8_t MOTOR_L_DIR1 = 5;
const uint8_t MOTOR_L_DIR2 = 6;

// 오른쪽 모터: L298N 채널B (IN3=10, IN4=11)
const uint8_t MOTOR_R_DIR1 = 10;
const uint8_t MOTOR_R_DIR2 = 11;

Motor motorL(MOTOR_L_DIR1, MOTOR_L_DIR2);
Motor motorR(MOTOR_R_DIR1, MOTOR_R_DIR2);

void setup() {
  Serial.begin(115200);

  motorL.begin();
  motorR.begin();

  Serial.println("READY");
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    handleCommand(line);
  }
}

// 파이썬 UI 통신 프로토콜 (PWM 없이 on/off 제어이므로 값의 부호만 사용)
//   "L,<speed>"  : 왼쪽 모터 방향 설정 (양수=정방향, 음수=역방향, 0=정지)
//   "R,<speed>"  : 오른쪽 모터 방향 설정 (양수=정방향, 음수=역방향, 0=정지)
//   "STOP"       : 두 모터 모두 정지
void handleCommand(const String &line) {
  if (line == "STOP") {
    motorL.stop();
    motorR.stop();
    Serial.println("ACK,STOP");
    return;
  }

  int commaIndex = line.indexOf(',');
  if (commaIndex <= 0) return;

  String target = line.substring(0, commaIndex);
  int speed = line.substring(commaIndex + 1).toInt();

  if (target == "L") {
    motorL.setSpeed(speed);
    Serial.print("ACK,L,");
    Serial.println(speed);
  } else if (target == "R") {
    motorR.setSpeed(speed);
    Serial.print("ACK,R,");
    Serial.println(speed);
  }
}
