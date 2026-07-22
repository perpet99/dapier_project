// --- 하드웨어 구성 ---
// - 모터 드라이버: L298N x1 (2채널, ENA/ENB=PWM 속도, IN1~IN4=방향)
// - 모터: 2개 (왼쪽/오른쪽)
// - 전원: 모터 구동 전원(18650 배터리)과 아두이노 로직 전원은 분리하여 공급 (GND는 공통)
// - 모터 제어는 별도 Motor 클래스(Motor.h / Motor.cpp)로 구현, 왼쪽/오른쪽 2개 객체 생성

#include "Motor.h"

// 왼쪽 모터: L298N 채널A (ENA=5 PWM, IN1=6, IN2=7)
const uint8_t MOTOR_L_PWM = 5;
const uint8_t MOTOR_L_DIR1 = 6;
const uint8_t MOTOR_L_DIR2 = 7;

// 오른쪽 모터: L298N 채널B (ENB=10 PWM, IN3=8, IN4=9)
const uint8_t MOTOR_R_PWM = 10;
const uint8_t MOTOR_R_DIR1 = 8;
const uint8_t MOTOR_R_DIR2 = 9;

Motor motorL(MOTOR_L_PWM, MOTOR_L_DIR1, MOTOR_L_DIR2);
Motor motorR(MOTOR_R_PWM, MOTOR_R_DIR1, MOTOR_R_DIR2);

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

// 파이썬 UI 통신 프로토콜
//   "L,<speed>"  : 왼쪽 모터 속도 설정 (speed: -255 ~ 255)
//   "R,<speed>"  : 오른쪽 모터 속도 설정 (speed: -255 ~ 255)
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
