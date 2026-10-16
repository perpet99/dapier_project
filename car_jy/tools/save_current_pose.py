#!/usr/bin/env python3
"""
현재 로봇 위치를 지정한 웨이포인트(예: home)로 XML에 자동 저장 (save_current_pose.py)
-------------------------------------------------------------------------
사용법:
  python3 tools/save_current_pose.py home
  python3 tools/save_current_pose.py point_a "지점 A"
"""

import os
import sys
import xml.etree.ElementTree as ET
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from get_current_pose import CurrentPoseReader
from run_xml_waypoints import DEFAULT_XML_PATHS, XML_PATH


def save_pose_to_xml(wp_id="home", wp_name=None, tag_id=None):
    rclpy.init()
    reader = CurrentPoseReader()
    x, y, yaw_deg, source = reader.get_pose(timeout_sec=3.0)
    reader.destroy_node()
    rclpy.shutdown()

    if x is None:
        print("\n❌ 현재 위치를 읽어올 수 없습니다. Nav2/AMCL이 실행 중인지 확인하세요.")
        return False

    xml_file = next((p for p in DEFAULT_XML_PATHS if os.path.exists(p)), XML_PATH)
    tree = ET.parse(xml_file)
    root = tree.getroot()

    found = False
    for wp in root.findall('waypoint'):
        if wp.get('id') == wp_id:
            found = True
            if wp_name:
                wp.set('name', wp_name)
            wp.find('x').text = f"{x:.2f}"
            wp.find('y').text = f"{y:.2f}"
            if wp.find('yaw') is not None:
                wp.find('yaw').text = f"{yaw_deg:.1f}"
            else:
                yaw_elem = ET.SubElement(wp, 'yaw')
                yaw_elem.text = f"{yaw_deg:.1f}"
            if tag_id is not None:
                if wp.find('tag_id') is not None:
                    wp.find('tag_id').text = str(tag_id)
                else:
                    tag_elem = ET.SubElement(wp, 'tag_id')
                    tag_elem.text = str(tag_id)
            break

    if not found:
        # 새로 추가
        new_wp = ET.SubElement(root, 'waypoint')
        new_wp.set('id', wp_id)
        new_wp.set('name', wp_name if wp_name else wp_id)
        x_elem = ET.SubElement(new_wp, 'x')
        x_elem.text = f"{x:.2f}"
        y_elem = ET.SubElement(new_wp, 'y')
        y_elem.text = f"{y:.2f}"
        yaw_elem = ET.SubElement(new_wp, 'yaw')
        yaw_elem.text = f"{yaw_deg:.1f}"
        wait_elem = ET.SubElement(new_wp, 'wait_time')
        wait_elem.text = "3.0"
        if tag_id is not None:
            tag_elem = ET.SubElement(new_wp, 'tag_id')
            tag_elem.text = str(tag_id)

    # 포맷팅 저장
    ET.indent(tree, space="    ", level=0)
    tree.write(xml_file, encoding='utf-8', xml_declaration=True)

    print("\n" + "=" * 65)
    print(f"💾 [웨이포인트 '{wp_id}' 좌표 저장 완료!]")
    print("=" * 65)
    print(f" • 대상 XML 파일:  {xml_file}")
    print(f" • 저장된 좌표:    X = {x:.2f} m, Y = {y:.2f} m, Yaw = {yaw_deg:.1f}°")
    print(f" • 데이터 출처:    {source}")
    print("=" * 65 + "\n")
    return True


def main():
    wp_id = sys.argv[1].strip() if len(sys.argv) >= 2 else "home"
    wp_name = sys.argv[2].strip() if len(sys.argv) >= 3 else None
    tag_id = 1 if wp_id == "home" else None
    save_pose_to_xml(wp_id, wp_name, tag_id)


if __name__ == '__main__':
    main()
