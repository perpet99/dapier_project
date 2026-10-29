#!/usr/bin/env python3
"""
RViz 화면에서 본 숫자를 그대로 웨이포인트(XML)에 직접 입력하는 도구 (set_waypoint.py)
-----------------------------------------------------------------------------
사용법:
  python3 tools/set_waypoint.py home <X> <Y> [Yaw각도]
  예시: python3 tools/set_waypoint.py home 2.5 2.0 -116.5
  예시: python3 tools/set_waypoint.py home -0.74 -0.2 0.0
"""

import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run_xml_waypoints import DEFAULT_XML_PATHS, XML_PATH


def set_waypoint(wp_id, x, y, yaw=None, tag_id=None):
    xml_file = next((p for p in DEFAULT_XML_PATHS if os.path.exists(p)), XML_PATH)
    tree = ET.parse(xml_file)
    root = tree.getroot()

    found = False
    for wp in root.findall('waypoint'):
        if wp.get('id') == wp_id:
            found = True
            wp.find('x').text = f"{float(x):.2f}"
            wp.find('y').text = f"{float(y):.2f}"
            if yaw is not None:
                if wp.find('yaw') is not None:
                    wp.find('yaw').text = f"{float(yaw):.1f}"
                else:
                    yaw_elem = ET.SubElement(wp, 'yaw')
                    yaw_elem.text = f"{float(yaw):.1f}"
            if tag_id is not None or wp_id == 'home':
                tid = tag_id if tag_id is not None else 1
                if wp.find('tag_id') is not None:
                    wp.find('tag_id').text = str(tid)
                else:
                    t_elem = ET.SubElement(wp, 'tag_id')
                    t_elem.text = str(tid)
            break

    if not found:
        new_wp = ET.SubElement(root, 'waypoint')
        new_wp.set('id', wp_id)
        new_wp.set('name', wp_id)
        x_elem = ET.SubElement(new_wp, 'x')
        x_elem.text = f"{float(x):.2f}"
        y_elem = ET.SubElement(new_wp, 'y')
        y_elem.text = f"{float(y):.2f}"
        yaw_elem = ET.SubElement(new_wp, 'yaw')
        yaw_elem.text = f"{float(yaw):.1f}" if yaw is not None else "0.0"
        wait_elem = ET.SubElement(new_wp, 'wait_time')
        wait_elem.text = "3.0"
        if tag_id is not None or wp_id == 'home':
            t_elem = ET.SubElement(new_wp, 'tag_id')
            t_elem.text = str(tag_id if tag_id is not None else 1)

    ET.indent(tree, space="    ", level=0)
    tree.write(xml_file, encoding='utf-8', xml_declaration=True)

    print("\n" + "=" * 65)
    print(f"✅ [웨이포인트 '{wp_id}' 좌표 직접 설정 완료!]")
    print("=" * 65)
    print(f" • 대상 XML 파일:  {xml_file}")
    print(f" • 설정된 좌표:    X = {float(x):.2f} m, Y = {float(y):.2f} m" + (f", Yaw = {float(yaw):.1f}°" if yaw is not None else ""))
    if wp_id == 'home':
        print(f" • 홈 전용 마커:   AprilTag ID #1")
    print("=" * 65 + "\n")
    return True


def main():
    if len(sys.argv) < 4:
        print("❌ 사용법: python3 tools/set_waypoint.py <웨이포인트ID> <X> <Y> [Yaw각도]")
        print("   예시: python3 tools/set_waypoint.py home 2.5 2.0")
        print("   예시: python3 tools/set_waypoint.py home -0.74 -0.2 0.0")
        sys.exit(1)

    wp_id = sys.argv[1].strip()
    x = float(sys.argv[2])
    y = float(sys.argv[3])
    yaw = float(sys.argv[4]) if len(sys.argv) >= 5 else None

    set_waypoint(wp_id, x, y, yaw)


if __name__ == '__main__':
    main()
