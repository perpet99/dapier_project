// ST3215 서보 ID 설정 스케치 (Waveshare General Driver for Robots, ESP32)
// - 새 ST3215는 출고 시 ID가 모두 1이므로, 버스에 "서보 1개만" 연결한 상태에서 ID를 바꾼다.
// - 서보 버스: S_RXD=GPIO18, S_TXD=GPIO19, 1Mbps (Waveshare 예제와 동일)
// - 시리얼 모니터: 115200bps, 줄 끝 "Newline"
//
// 명령 (시리얼 모니터에 입력)
//   SCAN              : ID 1~253 Ping 스캔, 응답한 서보의 ID/모드/전압/온도 출력
//   ID,<old>,<new>    : ID 변경 (EEPROM에 저장됨, 전원 꺼도 유지)
//   WHEEL,<id>        : 해당 서보를 바퀴(연속 회전) 모드로 EEPROM에 저장
//   SERVO,<id>        : 해당 서보를 위치(서보) 모드로 EEPROM에 되돌림
//   SPIN,<id>,<speed> : 바퀴 모드에서 회전 테스트 (speed: -3400~3400, 0=정지)
//   DIAG              : SCAN 실패 시 보레이트/핀 조합별 브로드캐스트 Ping 진단
//   HELP              : 명령 목록
//
// 권장 ID 배치 (car2.ino와 동일)
//   1 = 앞왼쪽(FL), 2 = 앞오른쪽(FR), 3 = 뒤왼쪽(RL), 4 = 뒤오른쪽(RR)
#include <SCServo.h>

#define S_RXD 18
#define S_TXD 19

SMS_STS st;

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  Serial.setTimeout(50);
  delay(1000);
  printHelp();
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    line.toUpperCase();
    if (line.length() > 0) handleCommand(line);
  }
}

void printHelp() {
  Serial.println("=== ST3215 ID SETUP ===");
  Serial.println("SCAN | DIAG | ID,<old>,<new> | WHEEL,<id> | SERVO,<id> | SPIN,<id>,<speed> | HELP");
  Serial.println("* ID 변경 시 버스에 서보 1개만 연결할 것");
}

// "A,1,2" 형식에서 index번째 정수값 추출 (index 1부터)
int argAt(const String &line, int index) {
  int start = 0;
  for (int i = 0; i < index; i++) {
    start = line.indexOf(',', start);
    if (start < 0) return -1;
    start++;
  }
  int end = line.indexOf(',', start);
  return (end < 0 ? line.substring(start) : line.substring(start, end)).toInt();
}

void scan() {
  Serial.println("SCAN 시작...");
  int found = 0;
  for (int id = 1; id <= 253; id++) {
    if (st.Ping(id) == id) {
      found++;
      Serial.printf("  ID=%d  MODE=%d(0=서보,1=바퀴)  V=%.1fV  T=%dC  POS=%d\n",
                    id, st.ReadMode(id), st.ReadVoltage(id) / 10.0,
                    st.ReadTemper(id), st.ReadPos(id));
    }
  }
  Serial.printf("SCAN 완료: %d개 발견\n", found);
}

// 스캔 실패 시 원인 진단: 보레이트 × 핀(정상/RX-TX 뒤바뀜) 조합마다 브로드캐스트 Ping(0xFE)
// 브로드캐스트 Ping은 ID와 무관하게 서보 1개가 응답하므로, 서보 1개만 연결한 상태에서 사용
void diag() {
  const long bauds[] = {1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400};
  const int pins[2][2] = {{S_RXD, S_TXD}, {S_TXD, S_RXD}};
  Serial.println("DIAG 시작 (브로드캐스트 Ping)...");
  bool any = false;
  for (int p = 0; p < 2; p++) {
    for (long baud : bauds) {
      Serial1.end();
      Serial1.begin(baud, SERIAL_8N1, pins[p][0], pins[p][1]);
      delay(20);
      int id = st.Ping(0xFE);
      Serial.printf("  RX=%d TX=%d %7ld bps : %s", pins[p][0], pins[p][1], baud,
                    id >= 0 ? "응답" : "-");
      if (id >= 0) { Serial.printf(" ID=%d", id); any = true; }
      Serial.println();
    }
  }
  // 원래 설정으로 복구
  Serial1.end();
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  if (!any) {
    Serial.println("DIAG: 전 조합 무응답 -> 서보 전원(DC 7~12V) / 커넥터 / 케이블 확인");
  }
}

void changeId(int oldId, int newId) {
  if (oldId < 1 || oldId > 253 || newId < 1 || newId > 253) {
    Serial.println("ERR: ID 범위 1~253");
    return;
  }
  if (st.Ping(oldId) != oldId) {
    Serial.printf("ERR: ID %d 응답 없음\n", oldId);
    return;
  }
  if (oldId != newId && st.Ping(newId) == newId) {
    Serial.printf("ERR: ID %d 이미 사용 중 (충돌)\n", newId);
    return;
  }
  st.unLockEprom(oldId);
  st.writeByte(oldId, SMS_STS_ID, newId);
  st.LockEprom(newId);
  delay(50);
  if (st.Ping(newId) == newId) {
    Serial.printf("OK: ID %d -> %d\n", oldId, newId);
  } else {
    Serial.println("ERR: 변경 후 응답 없음 (전원/배선 확인 후 SCAN)");
  }
}

void setMode(int id, int mode) {
  if (st.Ping(id) != id) {
    Serial.printf("ERR: ID %d 응답 없음\n", id);
    return;
  }
  st.unLockEprom(id);
  st.writeByte(id, SMS_STS_MODE, mode);
  st.LockEprom(id);
  delay(20);
  Serial.printf("OK: ID %d MODE=%d\n", id, st.ReadMode(id));
}

void handleCommand(const String &line) {
  if (line == "SCAN") {
    scan();
  } else if (line == "DIAG") {
    diag();
  } else if (line == "HELP") {
    printHelp();
  } else if (line.startsWith("ID,")) {
    changeId(argAt(line, 1), argAt(line, 2));
  } else if (line.startsWith("WHEEL,")) {
    setMode(argAt(line, 1), 1);
  } else if (line.startsWith("SERVO,")) {
    setMode(argAt(line, 1), 0);
  } else if (line.startsWith("SPIN,")) {
    int id = argAt(line, 1);
    int speed = constrain(argAt(line, 2), -3400, 3400);
    st.WheelMode(id);
    st.WriteSpe(id, speed, 50);
    Serial.printf("OK: ID %d SPEED=%d\n", id, speed);
  } else {
    Serial.println("ERR: 알 수 없는 명령 (HELP)");
  }
}
