import json
import aiohttp
from bot_modules.config import GEMINI_API_KEY


async def ask_gemini_intent(user_text: str, wps: dict) -> dict:
    """Gemini API를 호출하여 자연어 의도를 JSON 자율주행 계획으로 해석"""
    if not GEMINI_API_KEY:
        return None

    # system prompt 구성
    wp_summary = ", ".join([f"'{wp_id}'({info['name']})" for wp_id, info in wps.items()])
    system_instruction = f"""
너는 자율주행 이동 로봇(AMR)의 지능형 관제 AI 에이전트야.
사용자의 메시지를 분석하여 로봇이 수행해야 할 정교한 주행 코스 계획을 JSON 형태로 생성해줘.

등록된 장소 목록:
{wp_summary}

사용 가능한 액션 타입 (actions 배열 내부 객체):
1. {{"type": "move", "target": "지점ID"}} -> 등록된 특정 장소로 이동 (예: "home", "point_a", "point_b", "point_c", "point_d", "origin")
2. {{"type": "coords", "x": float, "y": float, "yaw": float}} -> 특정 임의 좌표로 이동
3. {{"type": "patrol"}} -> 전체 지점 순환 순찰
4. {{"type": "round_trip"}} -> 1초간 직진 후 180도 회전하여 원위치 복귀하는 왕복 테스트 주행
5. {{"type": "straight_run"}} -> 1초간 직진 후 즉시 정지하는 1초 직진 테스트 주행
6. {{"type": "stop"}} -> 긴급 정지
7. {{"type": "wait", "seconds": float}} -> 지정 시간 동안 정차 대기
8. {{"type": "chat"}} -> 단순 대화 및 인사 (주행 액션 없음)

반환할 JSON 구조 예시:
{{
  "reply": "사용자에게 전달할 친절한 한국어 안내 메시지",
  "actions": [
    {{"type": "move", "target": "point_b"}},
    {{"type": "wait", "seconds": 3.0}},
    {{"type": "move", "target": "home"}}
  ]
}}
반드시 다른 설명 없이 유효한 JSON만 반환해야 해.
"""

    models_to_try = ["gemini-3.5-flash-lite", "gemini-3.6-flash"]
    payload = {
        "contents": [
            {"parts": [{"text": user_text}]}
        ],
        "systemInstruction": {
            "parts": [{"text": system_instruction}]
        },
        "generationConfig": {
            "responseMimeType": "application/json",
            "temperature": 0.2
        }
    }

    for model_name in models_to_try:
        url = f"https://generativelanguage.googleapis.com/v1beta/models/{model_name}:generateContent?key={GEMINI_API_KEY}"
        try:
            async with aiohttp.ClientSession() as session:
                async with session.post(url, json=payload, timeout=aiohttp.ClientTimeout(total=8.0)) as resp:
                    if resp.status == 200:
                        data = await resp.json()
                        raw_text = data['candidates'][0]['content']['parts'][0]['text']
                        result = json.loads(raw_text)
                        return result
                    else:
                        err_msg = await resp.text()
                        print(f"⚠️ Gemini API ({model_name}) 오류 ({resp.status}): {err_msg[:80]}... 다음 모델 시도")
        except Exception as e:
            print(f"⚠️ Gemini API ({model_name}) 호출 실패: {e}")

    print("⚠️ 모든 Gemini 모델 호출 실패 (기존 키워드 Fallback 매칭 사용)")
    return None
