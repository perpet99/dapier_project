#include "Car.h"

// 장착 방향에 따른 회전 반전 여부 {FL, FR, RL, RR}
// 전진 명령 시 바퀴가 반대로 돌면 해당 값을 바꾼다
static const bool INVERT_DEFAULT[4] = {true, false, true, false};

Car::Car(uint8_t idFL, uint8_t idFR, uint8_t idRL, uint8_t idRR)
  : _ids{idFL, idFR, idRL, idRR}, _acc(50) {
  for (int i = 0; i < 4; i++) _invert[i] = INVERT_DEFAULT[i];
}

int Car::begin(HardwareSerial &bus, int8_t rxPin, int8_t txPin) {
  bus.begin(1000000, SERIAL_8N1, rxPin, txPin);
  _st.pSerial = &bus;
  delay(500);

  int found = 0;
  for (int i = 0; i < 4; i++) {
    if (ping(i)) found++;
    _st.WheelMode(_ids[i]);  // RAM에 적용 (EEPROM 저장은 servo_id_setup의 WHEEL 명령)
    _st.EnableTorque(_ids[i], 1);
    _st.WriteSpe(_ids[i], 0, _acc);
  }
  return found;
}

void Car::setWheels(int fl, int fr, int rl, int rr) {
  int speeds[4] = {fl, fr, rl, rr};
  for (int i = 0; i < 4; i++) {
    int s = constrain(speeds[i], -MAX_SPEED, MAX_SPEED);
    _st.WriteSpe(_ids[i], _invert[i] ? -s : s, _acc);
  }
}

void Car::drive(int left, int right) {
  setWheels(left, right, left, right);
}

void Car::stop() {
  setWheels(0, 0, 0, 0);
}

void Car::torque(bool enable) {
  for (int i = 0; i < 4; i++) _st.EnableTorque(_ids[i], enable ? 1 : 0);
}

bool Car::ping(uint8_t index) {
  return _st.Ping(_ids[index]) == _ids[index];
}

void Car::printStatus(Stream &out) {
  static const char *NAMES[4] = {"FL", "FR", "RL", "RR"};
  for (int i = 0; i < 4; i++) {
    if (_st.FeedBack(_ids[i]) == -1) {
      out.printf("STAT,%s,%d,NO_RESPONSE\n", NAMES[i], _ids[i]);
      continue;
    }
    // FeedBack으로 한 번에 읽은 값을 ID=-1로 꺼낸다 (cur: 원시값, ST3215 약 6.5mA/단위)
    out.printf("STAT,%s,%d,pos=%d,spd=%d,load=%d,V=%.1f,T=%d,cur=%d\n",
               NAMES[i], _ids[i], _st.ReadPos(-1), _st.ReadSpeed(-1), _st.ReadLoad(-1),
               _st.ReadVoltage(-1) / 10.0, _st.ReadTemper(-1), _st.ReadCurrent(-1));
  }
}
