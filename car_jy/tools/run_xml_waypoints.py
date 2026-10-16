#!/usr/bin/env python3
"""
XML 기반 자율주행 목표 좌표 관리 및 주행 실행기 (run_xml_waypoints.py)
------------------------------------------------------------------
- waypoints.xml 파일에서 좌표 목록을 읽어옵니다.
- 단일 지점 이동, 전체 순찰 주행(Patrol), XML 파일에 새 좌표 추가 기능을 제공합니다.
"""

import sys
import os
import time
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from navigate_to_coords import RobotNavigatorClient

DEFAULT_XML_PATHS = [
    os.path.expanduser("~/ros2_amr_ws/config/waypoints.xml"),
    os.path.expanduser("~/ros2_amr_ws/waypoints.xml"),
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/waypoints.xml"),
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "waypoints.xml"),
]
XML_PATH = next((p for p in DEFAULT_XML_PATHS if os.path.exists(p)), DEFAULT_XML_PATHS[0])


def load_waypoints(xml_file=None):
    """XML 파일에서 웨이포인트 딕셔너리 로드"""
    if xml_file is None:
        xml_file = next((p for p in DEFAULT_XML_PATHS if os.path.exists(p)), XML_PATH)

    if not os.path.exists(xml_file):
        print(f"❌ XML 파일을 찾을 수 없습니다: {xml_file}")
        return {}

    tree = ET.parse(xml_file)
    root = tree.getroot()

    waypoints = {}
    for wp in root.findall('waypoint'):
        wp_id = wp.get('id')
        name = wp.get('name', wp_id)
        x = float(wp.find('x').text)
        y = float(wp.find('y').text)
        yaw = float(wp.find('yaw').text) if wp.find('yaw') is not None else 0.0
        wait_time = float(wp.find('wait_time').text) if wp.find('wait_time') is not None else 3.0

        tag_id_elem = wp.find('tag_id')
        tag_id = int(tag_id_elem.text.strip()) if tag_id_elem is not None and tag_id_elem.text.strip().isdigit() else None

        waypoints[wp_id] = {
            'id': wp_id,
            'name': name,
            'x': x,
            'y': y,
            'yaw': yaw,
            'wait_time': wait_time,
            'tag_id': tag_id
        }
    return waypoints


def print_waypoints(waypoints):
    """웨이포인트 목록 예쁘게 출력"""
    print("\n" + "=" * 70)
    print(f"{'ID':<12} | {'장소 이름':<22} | {'X(m)':^8} | {'Y(m)':^8} | {'Yaw(°)':^8} | {'대기(초)':^6}")
    print("-" * 70)
    for wp_id, wp in waypoints.items():
        print(f"{wp_id:<12} | {wp['name']:<20} | {wp['x']:^8.2f} | {wp['y']:^8.2f} | {wp['yaw']:^8.1f} | {wp['wait_time']:^6.1f}")
    print("=" * 70 + "\n")


def find_waypoint(waypoints, query):
    """ID 또는 한글 이름으로 웨이포인트 똑똑하게 찾기"""
    q = query.strip().lower()
    # 1. 정확한 ID 일치
    if query in waypoints:
        return waypoints[query]
    for wp in waypoints.values():
        if wp['id'].lower() == q:
            return wp
    # 2. 이름에 키워드가 포함된 경우 (예: "작업대", "충전기", "거실")
    for wp in waypoints.values():
        if q in wp['name'].lower():
            return wp
    return None


def run_single(bot, waypoints, query):
    """단일 웨이포인트로 이동"""
    wp = find_waypoint(waypoints, query)
    if not wp:
        print(f"❌ XML에서 '{query}'에 해당하는 장소를 찾을 수 없습니다.")
        print("   등록된 장소 목록을 확인하세요:")
        print_waypoints(waypoints)
        return False

    print(f"\n📍 [{wp['name']}] (ID: {wp['id']}) 로 이동을 시작합니다...")
    success = bot.go_to(wp['x'], wp['y'], wp['yaw'])
    if success and wp['wait_time'] > 0:
        print(f"⏱️  해당 지점에서 {wp['wait_time']}초 동안 정차 대기합니다...")
        time.sleep(wp['wait_time'])
    return success


def run_patrol(bot, waypoints, loop=False):
    """XML에 등록된 모든 지점을 순차 순찰 주행"""
    items = list(waypoints.items())
    print(f"\n🚀 총 {len(items)}개 지점 순차 순찰 주행을 시작합니다!")
    
    cycle = 1
    while True:
        print(f"\n--- [순찰 사이클 {cycle}회차 시작] ---")
        for wp_id, wp in items:
            print(f"\n👉 다음 목적지: {wp['name']} (X={wp['x']}, Y={wp['y']})")
            success = bot.go_to(wp['x'], wp['y'], wp['yaw'])
            if not success:
                print(f"⚠️ {wp['name']} 이동 중 실패/취소되었습니다. 순찰을 중단합니다.")
                return False
            
            print(f"✅ {wp['name']} 도착 완료! {wp['wait_time']}초 대기...")
            time.sleep(wp['wait_time'])
        
        print(f"\n🎉 [사이클 {cycle}회차 순찰 완료!]")
        if not loop:
            break
        cycle += 1
        print("🔁 5초 후 다음 사이클 순찰을 시작합니다...")
        time.sleep(5.0)
    return True


def main():
    waypoints = load_waypoints()
    if not waypoints:
        return

    # 1. 인자가 있는 경우
    # 사용법: python3 run_xml_waypoints.py <ID> [--dock] (예: home --dock)
    # 또는:   python3 run_xml_waypoints.py patrol
    if len(sys.argv) >= 2:
        cmd = sys.argv[1].strip()
        do_dock = '--dock' in sys.argv

        if cmd == 'patrol':
            bot = RobotNavigatorClient()
            success = run_patrol(bot, waypoints, loop=False)
            sys.exit(0 if success else 1)
        elif cmd == 'loop':
            bot = RobotNavigatorClient()
            success = run_patrol(bot, waypoints, loop=True)
            sys.exit(0 if success else 1)
        elif find_waypoint(waypoints, cmd):
            if do_dock:
                from navigate_and_dock import run_nav_and_dock
                success = run_nav_and_dock(target_query=cmd)
                sys.exit(0 if success else 1)
            else:
                bot = RobotNavigatorClient()
                success = run_single(bot, waypoints, cmd)
                sys.exit(0 if success else 1)
        else:
            print(f"❌ 알 수 없는 명령이나 장소입니다: '{cmd}'")
            print_waypoints(waypoints)
            sys.exit(1)

    # 2. 대화형 메뉴
    print("=" * 65)
    print("        XML 기반 자율주행 웨이포인트 매니저")
    print("=" * 65)
    print_waypoints(waypoints)

    print("명령을 선택하세요:")
    print("  [ID 입력]  해당 지점으로 즉시 이동 (예: home, point_a)")
    print("  p          등록된 모든 지점 1회 순환 순찰 (Patrol)")
    print("  l          무한 반복 순찰 (Loop Patrol)")
    print("  q          종료")
    print("-" * 65)

    choice = input("입력: ").strip()
    if choice.lower() == 'q':
        return

    bot = RobotNavigatorClient()
    if choice.lower() == 'p':
        run_patrol(bot, waypoints, loop=False)
    elif choice.lower() == 'l':
        run_patrol(bot, waypoints, loop=True)
    elif choice in waypoints:
        run_single(bot, waypoints, choice)
    else:
        print(f"❌ 잘못된 입력입니다: {choice}")


if __name__ == '__main__':
    main()
