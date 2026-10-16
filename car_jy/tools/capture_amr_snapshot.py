#!/usr/bin/env python3
"""
tools/capture_amr_snapshot.py
AMR 카메라 실시간 스냅샷 캡처 및 VLM 물체 인식 도구

1. AMR 로봇 웹 서버(8080 포트)에서 최신 /snapshot.jpg 획득
2. /home/juyeon/ros2_amr_ws/snapshots/latest_object.jpg 에 저장
3. Antigravity의 view_file 도구 또는 외부 VLM API와 연계하여 물체 인식 수행
"""

import os
import sys
import time
import argparse
import urllib.request
import urllib.error
import json

_tools_dir = os.path.dirname(os.path.realpath(os.path.abspath(__file__)))
WORKSPACE_DIR = os.path.dirname(_tools_dir)
SNAPSHOT_DIR = os.path.join(WORKSPACE_DIR, "snapshots")
DEFAULT_OUTPUT = os.path.join(SNAPSHOT_DIR, "latest_object.jpg")

def capture_snapshot(hosts=["192.168.0.64", "localhost", "127.0.0.1"], port=8080, output_path=DEFAULT_OUTPUT, timeout=1.5):
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    for host in hosts:
        url = f"http://{host}:{port}/snapshot.jpg"
        # 1. 초고속(0.05초) curl 시도
        try:
            import subprocess
            res = subprocess.run(
                ["curl", "-s", "-m", str(timeout), url, "-o", output_path],
                check=True, timeout=timeout + 0.5
            )
            if os.path.exists(output_path) and os.path.getsize(output_path) > 1000:
                return {
                    "status": "success",
                    "source_url": url,
                    "file_path": output_path,
                    "bytes": os.path.getsize(output_path),
                    "timestamp": time.time()
                }
        except Exception:
            pass

        # 2. urllib 백업
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "AMR-VLM-Agent/1.0"})
            with urllib.request.urlopen(req, timeout=timeout) as response:
                if response.status == 200:
                    data = response.read()
                    with open(output_path, "wb") as f:
                        f.write(data)
                    return {
                        "status": "success",
                        "source_url": url,
                        "file_path": output_path,
                        "bytes": len(data),
                        "timestamp": time.time()
                    }
        except Exception:
            continue

    return {
        "status": "error",
        "message": f"모든 호스트({hosts}:{port})에서 스냅샷 캡처 실패. 스트리밍 프로그램(view_depthmap.py 또는 grasp_approach_stopper.py)이 실행 중인지 확인하세요."
    }

def main():
    parser = argparse.ArgumentParser(description="AMR 전방 카메라 스냅샷 캡처 도구")
    parser.add_argument("--host", default=None, help="로봇 IP 주소 (지정 안할 시 localhost, 192.168.0.64 순서로 탐색)")
    parser.add_argument("--port", type=int, default=8080, help="웹 스트림 포트 (기본: 8080)")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="저장할 이미지 파일 경로")
    parser.add_argument("--json", action="store_true", help="결과를 JSON 포맷으로 출력")

    args = parser.parse_args()

    hosts = [args.host] if args.host else ["192.168.0.64", "localhost", "127.0.0.1"]
    result = capture_snapshot(hosts=hosts, port=args.port, output_path=args.output)

    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        if result["status"] == "success":
            print(f"📸 [스냅샷 캡처 성공!]")
            print(f"   - 소스: {result['source_url']}")
            print(f"   - 저장 위치: {result['file_path']} ({result['bytes'] / 1024:.1f} KB)")
        else:
            print(f"❌ [스냅샷 캡처 실패]: {result.get('message')}")
            sys.exit(1)

if __name__ == "__main__":
    main()
