// --- 핀 설정 및 전역 변수 ---
// 왼쪽 모터
const int encoderPinL = 2; // 인터럽트 핀 (INT0)
const int motorL_PWM = 5;  // 속도
const int motorL_DIR = 6;  // 방향 (단방향 고정, LOW=전진)

// 오른쪽 모터
const int encoderPinR = 3; // 인터럽트 핀 (INT1)
const int motorR_PWM = 10;  // 속도
const int motorR_DIR = 11;  // 방향 (단방향 고정, LOW=전진)

volatile long encoderCountL = 0;
volatile long encoderCountR = 0;
long targetCount = 0;
bool isMoving = false;

const int MAX_SPEED = 200;      // 최대 PWM (0~255)
const int MIN_SPEED = 70;       // 최소 기동 PWM (스틱션 보정)
const int SLOWDOWN_RANGE = 50;  // 목표까지 이 카운트 이내로 들어오면 감속 시작

unsigned long lastLogTime = 0;
const unsigned long LOG_INTERVAL = 100; // encoderCount 로그 주기(ms)

void setup() {
  Serial.begin(115200);

  pinMode(encoderPinL, INPUT_PULLUP);
  pinMode(encoderPinR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinL), countEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinR), countEncoderR, CHANGE);

  pinMode(motorL_PWM, OUTPUT);
  pinMode(motorL_DIR, OUTPUT);
  pinMode(motorR_PWM, OUTPUT);
  pinMode(motorR_DIR, OUTPUT);
  stopMotors();

  Serial.println("\n[System Ready] Enter target distance (encoder counts).");
}

void loop() {
  // 0. encoderCount 로그 (이동 여부와 무관하게 주기적으로 출력, 엔코더 동작 확인용)
  if (millis() - lastLogTime >= LOG_INTERVAL) {
    long countL, countR;
    noInterrupts();
    countL = encoderCountL;
    countR = encoderCountR;
    interrupts();

    Serial.print("encoderCount L/R: ");
    Serial.print(countL);
    Serial.print(" / ");
    Serial.println(countR);

    lastLogTime = millis();
  }

  // 1. 목표 거리 입력 처리
  if (Serial.available() > 0) {
    long input = Serial.parseInt();
    if (input > 0) {
      noInterrupts();
      encoderCountL = 0;
      encoderCountR = 0;
      interrupts();

      targetCount = input;
      isMoving = true;

      Serial.print(">>> Target Set: ");
      Serial.println(targetCount);
    }
  }

  // 2. 목표 거리까지 두 모터 동시 전진
  if (isMoving) {
    long countL, countR;
    noInterrupts();
    countL = encoderCountL;
    countR = encoderCountR;
    interrupts();

    long remainingL = targetCount - countL;
    long remainingR = targetCount - countR;

    bool doneL = (remainingL <= 0);
    bool doneR = (remainingR <= 0);

    if (doneL && doneR) {
      stopMotors();
      isMoving = false;
      Serial.print("Done! Final Count L/R: ");
      Serial.print(countL);
      Serial.print(" / ");
      Serial.println(countR);
    } else {
      driveMotor(motorL_PWM, motorL_DIR, doneL ? 0 : remainingL);
      driveMotor(motorR_PWM, motorR_DIR, doneR ? 0 : remainingR);

      Serial.print("Remaining L/R: ");
      Serial.print(remainingL);
      Serial.print(" / ");
      Serial.print(remainingR);
      Serial.print("  Count L/R: ");
      Serial.print(countL);
      Serial.print(" / ");
      Serial.println(countR);
    }
  }
}

void countEncoderL() {
  encoderCountL++;
}

void countEncoderR() {
  encoderCountR++;
}

// 남은 거리(remaining)에 비례해 목표 근처에서 감속, 그 외엔 최대 속도로 구동
// remaining이 0 이하면 해당 모터는 정지
void driveMotor(int pwmPin, int dirPin, long remaining) {
  if (remaining <= 0) {
    analogWrite(pwmPin, 0);
    digitalWrite(dirPin, LOW);
    return;
  }

  int speed;
  if (remaining < SLOWDOWN_RANGE) {
    speed = map(remaining, 0, SLOWDOWN_RANGE, MIN_SPEED, MAX_SPEED);
  } else {
    speed = MAX_SPEED;
  }
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);

  analogWrite(pwmPin, speed);
  digitalWrite(dirPin, LOW);
}

void stopMotors() {
  analogWrite(motorL_PWM, 0); // PWM 듀티 0 = 전원 차단 (coast stop)
  digitalWrite(motorL_DIR, LOW);
  analogWrite(motorR_PWM, 0);
  digitalWrite(motorR_DIR, LOW);
}
