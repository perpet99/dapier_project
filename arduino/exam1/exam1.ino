// 루프를 돌면서 카운팅 값을 일정 주기로 시리얼로 전송하는 스케치
// (파이썬 시리얼 스레드와의 연결 확인용)

long counter = 0;

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 100; // 전송 주기(ms)

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (millis() - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = millis();
    Serial.println(counter++);
  }
}
