#!/usr/bin/env python3
"""
tools/update_vlm_result.py
AI VLM 분석 결과를 실행 중인 view_depthmap.py에 실시간 전송하여
영상 화면(오버레이 배너) 및 웹 대시보드에 즉시 반영하는 도구
"""

import sys
import argparse
import urllib.request
import json

def update_result(object_name, is_trash, details="", hosts=["192.168.0.64", "localhost", "127.0.0.1"], port=8080):
    payload = json.dumps({
        "object": object_name,
        "is_trash": is_trash,
        "details": details
    }, ensure_ascii=False).encode('utf-8')

    success = False
    for host in hosts:
        url = f"http://{host}:{port}/api/vlm_result"
        # 1. 초고속 curl POST 시도
        try:
            import subprocess
            res = subprocess.run([
                "curl", "-s", "-m", "1.0", "-X", "POST",
                "-H", "Content-Type: application/json; charset=utf-8",
                "-d", payload.decode('utf-8'),
                url
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=1.5)
            if res.returncode == 0 and b"success" in res.stdout:
                print(f"✅ [VLM 결과 전송 성공] {url} ➔ {object_name} (쓰레기: {is_trash})")
                return
        except Exception:
            pass

        # 2. urllib 백업
        try:
            req = urllib.request.Request(
                url,
                data=payload,
                headers={
                    "Content-Type": "application/json; charset=utf-8",
                    "User-Agent": "AMR-VLM-Agent/1.0"
                },
                method="POST"
            )
            with urllib.request.urlopen(req, timeout=1.0) as resp:
                if resp.status == 200:
                    success = True
                    print(f"✅ [VLM 결과 전송 성공] {url} ➔ {object_name} (쓰레기: {is_trash})")
        except Exception:
            continue

    if not success:
        print(f"⚠️ [전송 실패] 모든 호스트({hosts}:{port})에 연결할 수 없습니다. view_depthmap.py 실행 여부를 확인하세요.")

def main():
    parser = argparse.ArgumentParser(description="view_depthmap에 VLM 분석 결과 전송")
    parser.add_argument("--object", required=True, help="식별된 물체 명칭")
    parser.add_argument("--trash", required=True, choices=["true", "false", "unknown"], help="쓰레기 여부 (true/false)")
    parser.add_argument("--details", default="", help="상세 분석 및 권장 사항")
    parser.add_argument("--host", default=None, help="타겟 IP")
    parser.add_argument("--port", type=int, default=8080, help="타겟 포트")

    args = parser.parse_args()

    is_trash_val = None
    if args.trash.lower() == "true":
        is_trash_val = True
    elif args.trash.lower() == "false":
        is_trash_val = False

    hosts = [args.host] if args.host else ["192.168.0.64", "localhost", "127.0.0.1"]
    update_result(args.object, is_trash_val, args.details, hosts=hosts, port=args.port)

if __name__ == "__main__":
    main()
