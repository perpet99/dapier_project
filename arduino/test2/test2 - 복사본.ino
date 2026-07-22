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

// 엔코더 1회전당 펄스 수 (실제 엔코더/기어비에 맞게 반드시 보정할 것)
const float COUNTS_PER_REV = 700.0;

long targetCountL = 0;
long targetCountR = 0;
bool isMoving = false;

const int MAX_SPEED = 200;      // 최대 PWM (0~255)
const int MIN_SPEED = 70;       // 최소 기동 PWM (스틱션 보정)
const long STOP_TOLERANCE = 3;  // 목표 도달로 간주할 오차 허용 범위 (count)

// PID 게인 (실측 후 튜닝 필요)
double Kp = 2.0, Ki = 0.05, Kd = 0.5;

struct PIDState {
  double integral = 0;
  long prevError = 0;
};
PIDState pidL, pidR;

unsigned long lastControlTime = 0;
const unsigned long CONTROL_INTERVAL = 20; // PID 계산 주기(ms)

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

  Serial.println("\n[System Ready] Enter target angle (degrees).");
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

  // 1. 목표 각도 입력 처리
  if (Serial.available() > 0) {
    float angle = Serial.parseFloat();
    if (angle > 0) {
      noInterrupts();
      encoderCountL = 0;
      encoderCountR = 0;
      interrupts();

      targetCountL = lround(angle / 360.0 * COUNTS_PER_REV);
      targetCountR = targetCountL;

      pidL.integral = 0;
      pidL.prevError = targetCountL;
      pidR.integral = 0;
      pidR.prevError = targetCountR;

      isMoving = true;
      lastControlTime = millis();

      Serial.print(">>> Target Angle: ");
      Serial.print(angle);
      Serial.print(" deg -> Target Count: ");
      Serial.println(targetCountL);
    }
  }

  // 2. PID 제어로 목표 각도까지 두 모터 회전 후 정지
  if (isMoving && millis() - lastControlTime >= CONTROL_INTERVAL) {
    double dt = (millis() - lastControlTime) / 1000.0;
    lastControlTime = millis();

    long countL, countR;
    noInterrupts();
    countL = encoderCountL;
    countR = encoderCountR;
    interrupts();

    long errorL = targetCountL - countL;
    long errorR = targetCountR - countR;

    bool doneL = abs(errorL) <= STOP_TOLERANCE;
    bool doneR = abs(errorR) <= STOP_TOLERANCE;

    if (doneL && doneR) {
      stopMotors();
      isMoving = false;
      Serial.print("Done! Final Count L/R: ");
      Serial.print(countL);
      Serial.print(" / ");
      Serial.println(countR);
    } else {
      int speedL = doneL ? 0 : computePID(pidL, errorL, dt);
      int speedR = doneR ? 0 : computePID(pidR, errorR, dt);

      driveMotor(motorL_PWM, motorL_DIR, speedL);
      driveMotor(motorR_PWM, motorR_DIR, speedR);

      Serial.print("Error L/R: ");
      Serial.print(errorL);
      Serial.print(" / ");
      Serial.print(errorR);
      Serial.print("  Speed L/R: ");
      Serial.print(speedL);
      Serial.print(" / ");
      Serial.println(speedR);
    }
  }
}

void countEncoderL() {
  encoderCountL++;
}

void countEncoderR() {
  encoderCountR++;
}

// 오차(error)를 입력받아 PID 연산으로 출력 속도(PWM)를 산출
// 적분 와인드업 방지를 위해 integral 항은 범위를 제한
int computePID(PIDState &pid, long error, double dt) {
  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, -500, 500);

  double derivative = (dt > 0) ? (error - pid.prevError) / dt : 0;
  pid.prevError = error;

  double output = Kp * error + Ki * pid.integral + Kd * derivative;

  int speed = constrain((int)output, 0, MAX_SPEED);
  if (speed > 0 && speed < MIN_SPEED) speed = MIN_SPEED; // 스틱션 보정
  return speed;
}

void driveMotor(int pwmPin, int dirPin, int speed) {
  analogWrite(pwmPin, speed);
  digitalWrite(dirPin, LOW);
}

void stopMotors() {
  analogWrite(motorL_PWM, 0); // PWM 듀티 0 = 전원 차단 (coast stop)
  digitalWrite(motorL_DIR, LOW);
  analogWrite(motorR_PWM, 0);
  digitalWrite(motorR_DIR, LOW);
}
