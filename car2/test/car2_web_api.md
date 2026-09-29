# car2 REST API

`car2_web.py`가 제공하는 차량 조종 HTTP API. 웹 UI(`/`)도 이 API로 동작한다.

!! 첫 테스트는 반드시 차체를 들어 바퀴가 공중에 뜬 상태에서 진행 !!

## 서버 실행
```bash
python car2_web.py --port /dev/ttyS0                  # 기본 http://0.0.0.0:8766
python car2_web.py --port /dev/ttyS0 --speed 1200 --http-port 8766 --host 0.0.0.0
python car2_web.py --dry-run                           # 시리얼 없이 명령만 출력 (API 개발/테스트용)
```

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--port` | (필수, dry-run 제외) | 시리얼 포트. 예: `/dev/ttyS0`, `/dev/ttyUSB0` |
| `--dry-run` | 꺼짐 | 차량 없이 보낼 명령을 콘솔에 출력 |
| `--speed` | 800 | 시작 기본 속도 (step/s) |
| `--host` | `0.0.0.0` | HTTP 바인드 주소 |
| `--http-port` | 8766 | HTTP 포트 |

## 공통 사항
- Base URL: `http://<라즈베리파이 IP>:8766`
- 요청/응답 모두 JSON (`Content-Type: application/json`). 요청 body가 없으면 `{}`로 취급.
- 속도 단위는 step/s. 범위는 **200 ~ 3400**이며, 범위를 벗어난 값은 자동으로 잘린다.
- CORS 허용 (`Access-Control-Allow-Origin: *`) → 다른 도메인의 웹 페이지에서도 호출 가능.
- 인증 없음. 같은 네트워크에 있는 누구나 조종할 수 있으니 신뢰할 수 있는 네트워크에서만 사용.
- 여러 클라이언트가 동시에 보내면 마지막에 받은 명령이 적용된다.

## 안전 동작 (중요)
이동 명령은 **정해진 시간 동안만** 유효하다.

- `duration`을 생략하면 **0.35초** 동안만 움직이고 자동으로 정지한다.
  계속 움직이려면 0.35초보다 짧은 간격(권장 0.1초)으로 같은 명령을 다시 보낸다. 웹 UI도 버튼을 누르는 동안 0.1초마다 다시 보낸다.
- `duration`(초)을 지정하면 그 시간 동안 움직이고 정지한다. 최대 **10초**.
- 새 이동 명령이 오면 이전 명령을 대체하고 남은 시간도 새로 계산한다.
- 클라이언트가 끊겨 명령이 더 오지 않으면 기한이 지나는 대로 서버가 `STOP`을 보낸다. 서버가 죽더라도 차량 펌웨어(car2.ino)의 500ms 타임아웃으로 멈춘다.
- 서버 종료(Ctrl+C) 시 `STOP`을 보낸다.

## 엔드포인트 요약
| 메서드 | 경로 | 설명 |
|---|---|---|
| GET | `/api/status` | 현재 상태 조회 |
| POST | `/api/move` | 방향으로 이동 (전진/후진/제자리 좌·우회전) |
| POST | `/api/velocity` | 직진/회전 속도를 직접 지정해 이동 |
| POST | `/api/stop` | 즉시 정지 |
| POST | `/api/speed` | 기본 속도 변경 |

---

### GET `/api/status`
현재 상태를 조회한다.

**응답 예시**
```json
{
  "action": "전진",
  "linear": 800,
  "angular": 0,
  "speed": 800,
  "moving": true,
  "remaining": 0.26,
  "last_cmd": "V,800,0",
  "min_speed": 200,
  "max_speed": 3400,
  "speed_step": 200
}
```

| 필드 | 타입 | 설명 |
|---|---|---|
| `action` | string | 현재 동작: `정지`, `전진`, `후진`, `좌회전`, `우회전`, `속도지정` |
| `linear` | int | 현재 직진 속도 (양수 = 전진, 음수 = 후진). 정지 중이면 0 |
| `angular` | int | 현재 회전 속도 (양수 = 좌회전, 음수 = 우회전). 정지 중이면 0 |
| `speed` | int | 기본 속도. `/api/move`에서 `speed`를 생략하면 이 값을 쓴다 |
| `moving` | bool | 차량에 이동 명령을 보내는 중인지 여부 |
| `remaining` | float | 자동 정지까지 남은 시간(초) |
| `last_cmd` | string | 마지막으로 차량에 보낸 시리얼 명령 (`V,<linear>,<angular>` 또는 `STOP`) |
| `min_speed`, `max_speed`, `speed_step` | int | 속도 범위와 UI의 +/- 증감 단위 |

---

### POST `/api/move`
지정한 방향으로 이동한다.

**요청 body**
| 필드 | 필수 | 타입 | 설명 |
|---|---|---|---|
| `direction` | O | string | `forward`(전진), `backward`(후진), `left`(제자리 좌회전), `right`(제자리 우회전) |
| `speed` | | int | 이번 명령의 속도. 생략하면 기본 속도(`speed`) |
| `duration` | | float | 이동 시간(초), 0~10. 생략하면 0.35초 |

**예시**
```bash
# 1.5초 전진
curl -X POST http://localhost:8766/api/move -d '{"direction":"forward","duration":1.5}'

# 속도 1500으로 1초 제자리 좌회전
curl -X POST http://localhost:8766/api/move -d '{"direction":"left","speed":1500,"duration":1}'
```

---

### POST `/api/velocity`
직진/회전 속도를 직접 지정한다. 곡선 주행처럼 전진과 회전을 섞을 때 사용한다.

**요청 body**
| 필드 | 필수 | 타입 | 설명 |
|---|---|---|---|
| `linear` | | int | 직진 속도, -3400~3400 (양수 = 전진). 기본 0 |
| `angular` | | int | 회전 속도, -3400~3400 (양수 = 좌회전). 기본 0 |
| `duration` | | float | 이동 시간(초), 0~10. 생략하면 0.35초 |

`linear`와 `angular`가 모두 0이면 정지한다.

**예시**
```bash
# 전진하면서 오른쪽으로 곡선 주행 2초
curl -X POST http://localhost:8766/api/velocity -d '{"linear":800,"angular":-300,"duration":2}'
```

---

### POST `/api/stop`
즉시 정지한다. body는 필요 없다.

```bash
curl -X POST http://localhost:8766/api/stop
```

---

### POST `/api/speed`
기본 속도를 바꾼다. 이미 움직이는 중인 명령에는 적용되지 않고, 다음 `/api/move`부터 적용된다.

**요청 body** (둘 중 하나)
| 필드 | 타입 | 설명 |
|---|---|---|
| `speed` | int | 새 기본 속도 (200~3400) |
| `delta` | int | 현재 기본 속도에 더할 값 (예: `200`, `-200`) |

```bash
curl -X POST http://localhost:8766/api/speed -d '{"speed":1200}'
curl -X POST http://localhost:8766/api/speed -d '{"delta":-200}'
```

---

## 응답과 오류
- 성공한 POST 요청은 `200`과 함께 `{"ok": true, ...}`를 돌려준다. `...`에는 `/api/status`와 같은 필드가 들어간다.
- 오류

| 코드 | 경우 | 응답 예시 |
|---|---|---|
| 400 | 잘못된 JSON, 잘못된 `direction`, 숫자가 아닌 값, `/api/speed`에 `speed`/`delta` 없음 | `{"error": "direction 은 forward, backward, left, right 중 하나"}` |
| 404 | 없는 경로 | `{"error": "not found"}` |

## Python 클라이언트 예시
```python
import time
import requests

BASE = "http://192.168.0.10:8766"

# 정해진 시간만큼 이동 (서버가 알아서 정지)
requests.post(f"{BASE}/api/move", json={"direction": "forward", "duration": 1.0})
time.sleep(1.2)

# 조건이 만족될 때까지 계속 이동: 0.1초마다 다시 보냄
try:
    for _ in range(30):  # 약 3초
        requests.post(f"{BASE}/api/velocity", json={"linear": 600, "angular": 200})
        time.sleep(0.1)
finally:
    requests.post(f"{BASE}/api/stop")

print(requests.get(f"{BASE}/api/status").json())
```
