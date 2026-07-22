// L298N 모터 드라이버 1채널을 제어하는 클래스 (PWM 없이 방향 2핀만 사용)
// DIR1/DIR2 조합으로 정방향/역방향/정지만 전환하는 on/off 제어 방식이다.
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
  public:
    Motor(uint8_t pinDIR1, uint8_t pinDIR2);

    void begin();
    void setSpeed(int speed); // 양수=정방향, 음수=역방향, 0=정지 (속도 크기는 무시, on/off 제어)
    void stop();

  private:
    uint8_t _pinDIR1;
    uint8_t _pinDIR2;
};

#endif
