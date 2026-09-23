# car2 — ST3215 서보 4개로 만드는 4륜 자동차 (Waveshare General Driver for Robots)

## 구성
- `servo_id_setup/servo_id_setup.ino` : ST3215 ID 설정 / 버스 스캔 / 바퀴 모드 저장 / 단일 회전 테스트용 스케치
- `car2/car2.ino` : 자동차 메인 스케치. 시리얼 명령으로 4바퀴 제어, 명령 끊기면 자동 정지
- `car2/Car.h`, `car2/Car.cpp` : ST3215 4개를 바퀴 모드로 묶어 제어하는 클래스 (좌/우 반전 처리 포함)
- `test/car2_test.py` : PC(파이썬)에서 PING/상태/바퀴 방향/주행/타임아웃 자동 테스트 + 키보드 조종

## 바퀴 배치 / ID
```
        앞
  [1 FL]    [2 FR]
  [3 RL]    [4 RR]
        뒤
```
- 오른쪽 바퀴(FR, RR)는 왼쪽과 거울 방향으로 장착되므로 코드에서 부호를 반전한다 (`Car.cpp`의 `INVERT_DEFAULT`).

## 1. 준비 (Arduino IDE)
1. 보드 매니저에서 **esp32 by Espressif** 설치, 보드는 **ESP32 Dev Module** 선택.
2. SCServo 라이브러리 설치: https://files.waveshare.com/upload/7/78/SCServo.rar 압축 해제 → `SCServo` 폴더를 `Arduino/libraries/`에 복사.
3. 보드 USB(Type-C)는 CP2102 → 드라이버 필요 시 설치: https://files.waveshare.com/wiki/common/CP210x_USB_TO_UART.zip
4. 서보 전원은 보드 DC 입력(7~13V, **ST3215는 12V 권장**)으로 공급. USB 전원만으로는 서보가 돌지 않는다.

## 2. 서보 ID 설정 (서보마다 1회)
새 ST3215는 **ID가 모두 1**이다. 같은 ID가 버스에 둘 이상 있으면 통신이 깨지므로 **반드시 1개씩** 연결해서 설정한다.

1. `servo_id_setup.ino` 업로드 → 시리얼 모니터 115200bps, 줄 끝 "Newline".
2. 서보 1개만 연결 후 `SCAN` → `ID=1` 확인.
3. 바꿀 ID 입력: `ID,1,2` (FR용), `ID,1,3` (RL용), `ID,1,4` (RR용). FL은 1 그대로.
4. 바퀴 모드 저장(권장): `WHEEL,<id>` — car2.ino가 부팅 때마다 바퀴 모드로 전환하지만, 저장해 두면 전원 인가 직후 위치 모드로 튀는 일이 없다.
5. 회전 확인: `SPIN,<id>,800` → `SPIN,<id>,0`
6. 서보를 떼고 다음 서보로 반복. 끝나면 4개 모두 데이지 체인 연결 후 `SCAN` → ID 1,2,3,4 모두 보여야 함.

## 3. 자동차 스케치 업로드
1. `car2/car2.ino` 열기 (`Car.h`, `Car.cpp`가 탭으로 함께 열림) → 업로드.
2. 시리얼 모니터에서 `SERVOS,4/4`, `READY` 확인. 일부 FAIL이면 `PING,..,FAIL` 줄로 어느 바퀴인지 표시됨.
3. 시리얼 모니터는 닫고 파이썬 테스트 진행 (포트 점유 충돌 방지).

### 시리얼 명령 (115200bps, 속도 단위 step/s, -3400~3400)
| 명령 | 설명 |
|---|---|
| `V,<linear>,<angular>` | 직진 + 회전 (angular 양수 = 좌회전) |
| `D,<left>,<right>` | 좌/우 속도 |
| `L,<speed>` / `R,<speed>` | 왼쪽 / 오른쪽만 |
| `W,<fl>,<fr>,<rl>,<rr>` | 바퀴 개별 (점검용) |
| `ACC,<0~254>` | 가감속 (0 = 즉시) |
| `STOP` | 정지 |
| `STAT` | 바퀴별 위치/속도/부하/전압/온도/전류 |
| `PING` | 4개 서보 응답 확인 |
| `TORQUE,<0/1>` | 토크 끄기/켜기 |

- **안전 타임아웃**: 이동 명령 후 500ms 동안 새 명령이 없으면 자동 정지(`TIMEOUT,STOP`). 계속 움직이려면 100ms 정도 주기로 명령을 재전송해야 한다 (`CMD_TIMEOUT_MS`로 조정).

## 4. 파이썬 테스트
```
pip install pyserial
cd car2/test
python car2_test.py --port /dev/ttyUSB0          # 전체 (ping → stat → wheels → drive → timeout)
python car2_test.py --port /dev/ttyUSB0 --test wheels
python car2_test.py --port /dev/ttyUSB0 --keyboard
```
- **첫 테스트는 차체를 들어 바퀴를 공중에 띄운 상태로.**
- `wheels` 단계에서 각 바퀴가 "차량 전진 방향"으로 도는지 눈으로 확인. 반대로 도는 바퀴는 `Car.cpp`의 `INVERT_DEFAULT` 해당 값을 반대로 바꾼다.

## 문제 해결
- `SERVOS,0/4` : DC 전원(7~13V) 미인가, 서보 케이블 방향/커넥터 확인, SCServo 라이브러리 버전 확인.
- 일부만 응답 : ID 중복 가능성 → 하나씩 연결해 `SCAN`.
- 바퀴 모드인데 일정 각도만 움직이고 멈춤 : 모드가 0(위치 모드) → `WHEEL,<id>`로 저장 후 재부팅.
- 좌/우 회전 방향이 반대 : `V`의 angular 부호를 반대로 보내거나 `car2.ino`의 `V` 처리에서 부호 교환.
- 업로드 실패 : 업로드 중 서보 전원이 켜져 있어도 무방하지만, 포트를 다른 프로그램이 쓰고 있지 않은지 확인.
