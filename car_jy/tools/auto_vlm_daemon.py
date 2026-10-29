#!/usr/bin/env python3
"""
tools/auto_vlm_daemon.py
AMR 실시간 자동 물체 감지 ➔ VLM AI 분석 ➔ 대시보드 동기화 파이프라인 데몬

동작 방식:
1. 로봇(192.168.0.64:8080 또는 localhost)의 /api/vlm_result 상태를 주기적으로 감시합니다.
2. 로봇에서 물체가 자동 감지되어 캡처(status == 'captured')되면 즉시 감지합니다.
3. 로봇으로부터 최신 고화질 스냅샷을 획득합니다.
4. Antigravity AI(agy) 멀티모달 시각 지능을 호출하여 물체 식별 및 쓰레기 여부를 자동 판별합니다.
5. 분석 결과를 로봇의 /api/vlm_result로 역전송하여 영상 배너와 웹 대시보드를 실시간 갱신합니다.
"""

import os
import sys
import time
import json
import re
import argparse
import subprocess
import urllib.request
import urllib.error

_tools_dir = os.path.dirname(os.path.realpath(os.path.abspath(__file__)))
WORKSPACE_DIR = os.path.dirname(_tools_dir)
SNAPSHOT_DIR = os.path.join(WORKSPACE_DIR, "snapshots")
SNAPSHOT_FILE = os.path.join(SNAPSHOT_DIR, "latest_object.jpg")

# agy 실행 파일 탐색
AGY_CANDIDATES = [
    os.environ.get("ANTIGRAVITY_AGENTAPI_EXE", ""),
    os.path.expanduser("~/.local/bin/agy"),
    os.path.expanduser("~/.gemini/antigravity-cli/bin/agy"),
    "/usr/local/bin/agy",
    "agy"
]

def find_agy_bin():
    import shutil
    for path in AGY_CANDIDATES:
        if path and (os.path.isfile(path) and os.access(path, os.X_OK)):
            return path
        if path and shutil.which(path):
            return shutil.which(path)
    return "agy"

def query_robot_status(host, port, timeout=1.5):
    url = f"http://{host}:{port}/api/vlm_result"
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "AMR-Auto-VLM-Daemon/1.0"})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            if resp.status == 200:
                return json.loads(resp.read().decode('utf-8'))
    except Exception:
        return None
    return None

def fetch_snapshot(host, port, output_path=SNAPSHOT_FILE, timeout=2.0):
    url = f"http://{host}:{port}/snapshot.jpg"
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    try:
        # curl 시도
        res = subprocess.run(
            ["curl", "-s", "-m", str(timeout), url, "-o", output_path],
            check=True, timeout=timeout + 1.0
        )
        if os.path.exists(output_path) and os.path.getsize(output_path) > 1000:
            return True
    except Exception:
        pass

    try:
        req = urllib.request.Request(url, headers={"User-Agent": "AMR-Auto-VLM-Daemon/1.0"})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            if resp.status == 200:
                with open(output_path, "wb") as f:
                    f.write(resp.read())
                return True
    except Exception:
        pass
    return False

def run_ai_vlm_analysis(image_path, agy_bin):
    """agy CLI를 통해 이미지 VLM 분석 수행"""
    if not os.path.exists(image_path):
        return None

    prompt = (
        f"{image_path} 사진을 보고 로봇 전방에 감지된 물체(장애물)를 정밀 분석해줘. "
        "반드시 아래 JSON 형식만 출력해줘:\n"
        "{\n"
        '  "object": "<한국어 물체명 (영문명)>",\n'
        '  "is_trash": <true 또는 false>,\n'
        '  "details": "<형태/색상 특징 및 로봇 파지 또는 회피 권장사항>"\n'
        "}"
    )

    cmd = [agy_bin, "--dangerously-skip-permissions", "-p", prompt]
    try:
        proc = subprocess.run(
            cmd,
            cwd=WORKSPACE_DIR,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=45.0
        )
        if proc.returncode != 0:
            print(f"⚠️ [AI VLM 실행 오류] 코드: {proc.returncode}, stderr: {proc.stderr[:100]}")
            return None

        out = proc.stdout
        # JSON 블록 파싱 (```json ... ``` 또는 {...})
        json_match = re.search(r'\{[\s\S]*?"object"[\s\S]*?\}', out)
        if json_match:
            try:
                parsed = json.loads(json_match.group(0))
                return parsed
            except Exception as e:
                print(f"⚠️ JSON 파싱 실패: {e}\n원본: {json_match.group(0)}")
        else:
            print(f"⚠️ JSON 형식을 찾지 못함: {out[:200]}")
    except subprocess.TimeoutExpired:
        print("⏳ [AI VLM 분석 시간 초과 (45초)]")
    except Exception as e:
        print(f"⚠️ [AI VLM 실행 중 예외] {e}")

    return None

def update_robot_vlm(host, port, vlm_data):
    url = f"http://{host}:{port}/api/vlm_result"
    payload = json.dumps(vlm_data, ensure_ascii=False).encode('utf-8')
    try:
        # curl POST
        res = subprocess.run([
            "curl", "-s", "-m", "2.0", "-X", "POST",
            "-H", "Content-Type: application/json; charset=utf-8",
            "-d", payload.decode('utf-8'),
            url
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=2.5)
        if res.returncode == 0 and b"success" in res.stdout:
            return True
    except Exception:
        pass

    try:
        req = urllib.request.Request(
            url,
            data=payload,
            headers={"Content-Type": "application/json; charset=utf-8"},
            method="POST"
        )
        with urllib.request.urlopen(req, timeout=2.0) as resp:
            return resp.status == 200
    except Exception:
        return False

def run_daemon(hosts=["192.168.0.64", "127.0.0.1", "localhost"], port=8080, poll_interval=1.0):
    agy_bin = find_agy_bin()
    print("=" * 68)
    print("🤖 [AMR 실시간 자동 AI VLM 파이프라인 데몬 가동]")
    print(f"   - 대상 호스트: {hosts}:{port}")
    print(f"   - AI 분석 엔진: {agy_bin}")
    print(f"   - 스냅샷 저장소: {SNAPSHOT_FILE}")
    print("   - 동작 모드: 물체 감지 시 자동 캡처 ➔ AI 분석 ➔ 화면 실시간 동기화")
    print("=" * 68 + "\n")

    active_host = None
    last_processed_ts = 0.0

    while True:
        try:
            # 1. 활성 호스트 탐색
            status_data = None
            if active_host:
                status_data = query_robot_status(active_host, port)
                if not status_data:
                    active_host = None

            if not active_host:
                for h in hosts:
                    s = query_robot_status(h, port)
                    if s:
                        active_host = h
                        status_data = s
                        print(f"🔗 [연결 확인] 로봇 웹 서버 연결됨: http://{active_host}:{port}")
                        break

            if not active_host or not status_data:
                time.sleep(2.0)
                continue

            status = status_data.get("status")
            ts = float(status_data.get("timestamp", 0.0))

            # 2. 신규 캡처 감지 (status == 'captured' 이고 새로운 타임스탬프)
            if status == "captured" and (ts > last_processed_ts):
                print(f"\n⚡ [신규 스냅샷 감지] Timestamp: {ts} (거리: ~{status_data.get('dist_cm', 0)}cm)")
                last_processed_ts = ts

                # (1) 스냅샷 획득
                print(f"📥 [1/3] 로봇으로부터 스냅샷 다운로드 중...")
                if not fetch_snapshot(active_host, port):
                    print("❌ 스냅샷 획득 실패, 다음 주기에 재시도합니다.")
                    continue

                print(f"✅ 스냅샷 획득 완료 ({os.path.getsize(SNAPSHOT_FILE)} bytes)")

                # (2) AI VLM 분석 호출
                print(f"🧠 [2/3] AI VLM 분석 모델 실행 중... (Antigravity VLM)")
                t_start = time.time()
                analysis = run_ai_vlm_analysis(SNAPSHOT_FILE, agy_bin)
                t_elapsed = time.time() - t_start

                if analysis:
                    obj_name = analysis.get("object", "알 수 없는 물체")
                    is_trash = analysis.get("is_trash", False)
                    details = analysis.get("details", "")

                    trash_label = "🗑️ [쓰레기]" if is_trash else "💎 [유효 물품/공구]"
                    print(f"✨ [3/3] AI 분석 완료 ({t_elapsed:.1f}초 소요):")
                    print(f"    - 물체: {obj_name}")
                    print(f"    - 판정: {trash_label}")
                    print(f"    - 내용: {details}")

                    # (3) 로봇 대시보드로 결과 전송
                    payload = {
                        "object": obj_name,
                        "is_trash": is_trash,
                        "details": details,
                        "dist_cm": status_data.get("dist_cm", 0),
                        "status": "completed"
                    }
                    if update_robot_vlm(active_host, port, payload):
                        print(f"📡 ➔ 로봇 화면 및 웹 대시보드 실시간 반영 성공!\n")
                    else:
                        print("⚠️ 로봇으로 분석 결과 전송 실패\n")
                else:
                    print("⚠️ AI VLM 분석 결과를 얻지 못했습니다.\n")

            time.sleep(poll_interval)

        except KeyboardInterrupt:
            print("\n🛑 [데몬 종료] 사용자에 의해 중단되었습니다.")
            break
        except Exception as e:
            time.sleep(2.0)

def main():
    parser = argparse.ArgumentParser(description="AMR 자동 AI VLM 분석 파이프라인 데몬")
    parser.add_argument("--host", default="192.168.0.64", help="로봇 IP (기본: 192.168.0.64)")
    parser.add_argument("--port", type=int, default=8080, help="로봇 웹 서버 포트 (기본: 8080)")
    parser.add_argument("--interval", type=float, default=1.0, help="폴링 주기(초) (기본: 1.0)")
    args = parser.parse_args()

    hosts = [args.host, "127.0.0.1", "localhost"]
    unique_hosts = list(dict.fromkeys(hosts))
    run_daemon(unique_hosts, args.port, args.interval)

if __name__ == "__main__":
    main()
