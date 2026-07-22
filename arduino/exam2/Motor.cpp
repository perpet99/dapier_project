#include "Motor.h"

Motor::Motor(uint8_t pinDIR1, uint8_t pinDIR2)
  : _pinDIR1(pinDIR1), _pinDIR2(pinDIR2) {
}

void Motor::begin() {
  pinMode(_pinDIR1, OUTPUT);
  pinMode(_pinDIR2, OUTPUT);
  stop();
}

void Motor::setSpeed(int speed) {
  if (speed > 0) {
    digitalWrite(_pinDIR1, HIGH);
    digitalWrite(_pinDIR2, LOW);
  } else if (speed < 0) {
    digitalWrite(_pinDIR1, LOW);
    digitalWrite(_pinDIR2, HIGH);
  } else {
    stop();
  }
}

void Motor::stop() {
  digitalWrite(_pinDIR1, LOW);
  digitalWrite(_pinDIR2, LOW);
}
