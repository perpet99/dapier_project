---
name: vlm-object-inspector
description: >-
  Captures a front-facing camera snapshot from the autonomous mobile robot (AMR)
  and inspects or identifies the detected obstacle or object using multimodal vision intelligence.
  Use this skill whenever the user asks what object is in front of the robot, to identify or classify
  an obstacle, to check what was detected inside the bounding box, or to inspect the camera view.
---

# AMR 전방 물체 캡처 및 VLM 시각 지능 식별 스킬

이 스킬은 AMR 로봇이 이동 중이거나 정지했을 때, 전방 카메라 영상을 캡처하고 멀티모달 시각 지능(VLM)을 활용하여 박스/통로 안의 물체가 무엇인지 정밀하게 판별 및 분석하는 절차를 안내합니다.

## 사용 시점
- "앞에 있는 물체가 뭔지 확인해줘"
- "박스 안에 물체 뭐 있어?"
- "전방에 감지된 장애물 분석해줘"
- "스냅샷 찍어서 물체 식별해줘"
- 자율 전진 정지 후 감지된 물체 확인 요청 시

---

## 실행 절차

### 1단계: 전방 카메라 최신 스냅샷 캡처
`tools/capture_amr_snapshot.py` 스크립트를 실행하여 로봇 웹 서버로부터 실시간 고화질 스냅샷을 `/home/juyeon/ros2_amr_ws/snapshots/latest_object.jpg`에 다운로드합니다.

```bash
python3 /home/juyeon/ros2_amr_ws/tools/capture_amr_snapshot.py
```
> 로봇 IP가 다른 경우: `python3 /home/juyeon/ros2_amr_ws/tools/capture_amr_snapshot.py --host 192.168.0.64`

### 2단계: 이미지 시각 분석 (`view_file` 호출)
캡처가 완료되면 AI 에이전트는 즉시 `view_file` 도구를 호출하여 저장된 이미지를 직접 육안 분석합니다.

```python
view_file(AbsolutePath="/home/juyeon/ros2_amr_ws/snapshots/latest_object.jpg")
```

### 3단계: 분석 및 판정 항목
이미지를 분석한 후 다음 항목을 포함하여 간결하고 명확하게 답변합니다:

1. **식별된 물체 명칭**: (예: 원통형 리튬이온 배터리, 택배 상자, 생수병, 스카치테이프 등)
2. **외형적 특징**: 색상, 재질, 형태, 인쇄된 마크/로고
3. **쓰레기 여부 판정**: 
   - 🗑️ **[쓰레기]**: 구겨진 휴지, 폐마스크, 비닐 스크랩, 깨진 파편 등
   - 💎 **[유효 물품/부품 (쓰레기 아님)]**: 배터리, 공구, 정상 완제품, 부품 보관통 등
4. **물리적 위치 및 추정 거리**: (RGB 화면 및 뎁스맵 배너 태그에 표시된 거리, 예: ~23cm)
5. **그리퍼 파지/회피 판단**: 로봇 암으로 집을 수 있는 형태인지, 아니면 우회 회피해야 하는 고정 장애물인지 제안

### 4단계: `view_depthmap` 실시간 화면 동기화
분석이 끝나면 즉시 `tools/update_vlm_result.py`를 호출하여 로봇의 실시간 영상 오버레이 및 웹 대시보드에 분석 결과를 즉시 전송 반영합니다:

```bash
python3 /home/juyeon/ros2_amr_ws/tools/update_vlm_result.py --object "<물체명>" --trash <true|false> --details "<요약 권장사항>"
```
