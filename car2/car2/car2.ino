// --- 하드웨어 구성 ---
// - 보드: Waveshare General Driver for Robots (ESP32-WROOM-32)
// - 모터: ST3215 시리얼 버스 서보 4개 (바퀴 모드 = 연속 회전)
//         ID 1=앞왼쪽(FL), 2=앞오른쪽(FR), 3=뒤왼쪽(RL), 4=뒤오른쪽(RR)
//         (ID 설정은 servo_id_setup.ino로 먼저 진행)
// - 서보 버스: 보드의 ST3215 커넥터 (S_RXD=GPIO18, S_TXD=GPIO19, 1Mbps)
// - 전원: 보드 DC 입력 7~13V (ST3215는 12V 권장, 3S 리튬 배터리 등)
// - 제어: USB 시리얼(115200bps) 텍스트 명령
//
// 시리얼 프로토콜 (줄 끝 '\n', 속도 단위 step/s, 범위 -3400~3400)
//   "L,<speed>"                : 왼쪽 바퀴(FL,RL) 속도
//   "R,<speed>"                : 오른쪽 바퀴(FR,RR) 속도
//   "D,<left>,<right>"         : 좌/우 동시 설정
//   "V,<linear>,<angular>"     : 직진/회전 속도 (angular 양수 = 좌회전)
//   "W,<fl>,<fr>,<rl>,<rr>"    : 바퀴별 개별 속도 (배선/방향 점검용)
//   "ACC,<0~254>"              : 가감속 설정 (0=즉시)
//   "STOP"                     : 정지
//   "STAT"                     : 바퀴별 상태(위치/속도/부하/전압/온도/전류) 출력
//   "PING"                     : 4개 서보 응답 확인
//   "TORQUE,<0|1>"             : 토크 끄기/켜기 (0이면 손으로 바퀴가 돈다)
// 응답: "ACK,<명령>" / "ERR,<사유>"
//
// 안전: 마지막 이동 명령 후 CMD_TIMEOUT_MS 동안 새 명령이 없으면 자동 정지
//       (PC 연결이 끊겨도 차가 계속 달리지 않도록)
#include "Car.h"

#define S_RXD 18
#define S_TXD 19

const uint8_t ID_FL = 1;
const uint8_t ID_FR = 2;
const uint8_t ID_RL = 3;
const uint8_t ID_RR = 4;

const unsigned long CMD_TIMEOUT_MS = 500;

Car car(ID_FL, ID_FR, ID_RL, ID_RR);

int leftSpeed = 0;
int rightSpeed = 0;
bool moving = false;
unsigned long lastMoveCmd = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);

  int found = car.begin(Serial1, S_RXD, S_TXD);
  Serial.printf("SERVOS,%d/4\n", found);
  if (found < 4) printPing();
  Serial.println("READY");
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    line.toUpperCase();
    if (line.length() > 0) handleCommand(line);
  }

  if (moving && millis() - lastMoveCmd > CMD_TIMEOUT_MS) {
    stopCar();
    Serial.println("TIMEOUT,STOP");
  }
}

// "A,1,2" 형식에서 index번째 정수값 추출 (index 1부터), 없으면 def
int argAt(const String &line, int index, int def) {
  int start = 0;
  for (int i = 0; i < index; i++) {
    start = line.indexOf(',', start);
    if (start < 0) return def;
    start++;
  }
  int end = line.indexOf(',', start);
  return (end < 0 ? line.substring(start) : line.substring(start, end)).toInt();
}

void applyDrive(int left, int right) {
  leftSpeed = constrain(left, -Car::MAX_SPEED, Car::MAX_SPEED);
  rightSpeed = constrain(right, -Car::MAX_SPEED, Car::MAX_SPEED);
  car.drive(leftSpeed, rightSpeed);
  moving = (leftSpeed != 0 || rightSpeed != 0);
  lastMoveCmd = millis();
}

void stopCar() {
  car.stop();
  leftSpeed = rightSpeed = 0;
  moving = false;
}

void printPing() {
  static const char *NAMES[4] = {"FL", "FR", "RL", "RR"};
  for (int i = 0; i < 4; i++) {
    Serial.printf("PING,%s,%d,%s\n", NAMES[i], car.id(i), car.ping(i) ? "OK" : "FAIL");
  }
}

void handleCommand(const String &line) {
  int comma = line.indexOf(',');
  String cmd = comma < 0 ? line : line.substring(0, comma);

  if (cmd == "STOP") {
    stopCar();
  } else if (cmd == "L") {
    applyDrive(argAt(line, 1, 0), rightSpeed);
  } else if (cmd == "R") {
    applyDrive(leftSpeed, argAt(line, 1, 0));
  } else if (cmd == "D") {
    applyDrive(argAt(line, 1, 0), argAt(line, 2, 0));
  } else if (cmd == "V") {
    int linear = argAt(line, 1, 0);
    int angular = argAt(line, 2, 0);
    applyDrive(linear - angular, linear + angular);
  } else if (cmd == "W") {
    car.setWheels(argAt(line, 1, 0), argAt(line, 2, 0), argAt(line, 3, 0), argAt(line, 4, 0));
    moving = true;
    lastMoveCmd = millis();
  } else if (cmd == "ACC") {
    car.setAccel(constrain(argAt(line, 1, 0), 0, 254));
  } else if (cmd == "STAT") {
    car.printStatus(Serial);
  } else if (cmd == "PING") {
    printPing();
  } else if (cmd == "TORQUE") {
    stopCar();
    car.torque(argAt(line, 1, 1) != 0);
  } else {
    Serial.print("ERR,UNKNOWN,");
    Serial.println(line);
    return;
  }
  Serial.print("ACK,");
  Serial.println(line);
}
