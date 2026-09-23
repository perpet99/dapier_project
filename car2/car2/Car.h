// ST3215 서보 4개(바퀴 모드)로 구동하는 4륜 차동(스키드 스티어) 자동차 제어 클래스
// - 서보 ID: FL(앞왼쪽), FR(앞오른쪽), RL(뒤왼쪽), RR(뒤오른쪽)
// - 오른쪽 바퀴는 왼쪽과 거울 방향으로 장착되므로 부호를 반전해서 보낸다 (INVERT_* 로 조정)
#ifndef CAR_H
#define CAR_H

#include <Arduino.h>
#include <SCServo.h>

class Car {
  public:
    enum { MAX_SPEED = 3400 };           // ST3215 최대 속도 (step/s, 4096 step = 1회전)

    Car(uint8_t idFL, uint8_t idFR, uint8_t idRL, uint8_t idRR);

    // 서보 버스 초기화 + 4개 서보를 바퀴 모드로 전환. 응답한 서보 개수 반환
    int begin(HardwareSerial &bus, int8_t rxPin, int8_t txPin);

    void setWheels(int fl, int fr, int rl, int rr); // 바퀴별 속도 (양수=차량 전진 방향)
    void drive(int left, int right);                // 좌/우 측 속도
    void stop();
    void setAccel(uint8_t acc) { _acc = acc; }      // 가감속 (0=즉시, 1~254, 클수록 빠름)
    void torque(bool enable);

    bool ping(uint8_t index);                        // index: 0=FL,1=FR,2=RL,3=RR
    void printStatus(Stream &out);                   // 바퀴별 위치/속도/전압/온도/부하 출력

    uint8_t id(uint8_t index) const { return _ids[index]; }

  private:
    SMS_STS _st;
    uint8_t _ids[4];
    bool _invert[4];
    uint8_t _acc;
};

#endif
