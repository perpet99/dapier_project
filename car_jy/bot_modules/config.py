import os
import xml.etree.ElementTree as ET

# .env 파일 로드
_env_path = os.path.expanduser("~/ros2_amr_ws/.env")
if os.path.exists(_env_path):
    with open(_env_path, "r", encoding="utf-8") as _f:
        for _line in _f:
            _line = _line.strip()
            if _line.startswith("DISCORD_BOT_TOKEN="):
                os.environ["DISCORD_BOT_TOKEN"] = _line.split("=", 1)[1].strip("\"'")
            elif _line.startswith("GEMINI_API_KEY="):
                os.environ["GEMINI_API_KEY"] = _line.split("=", 1)[1].strip("\"'")

DISCORD_TOKEN = os.environ.get("DISCORD_BOT_TOKEN", "YOUR_BOT_TOKEN_HERE")
GEMINI_API_KEY = os.environ.get("GEMINI_API_KEY", "")

# Waypoints XML 및 실행 스크립트 경로 자동 탐색
_BASE_DIR = os.path.dirname(os.path.abspath(__file__))

DEFAULT_XML_PATHS = [
    os.path.expanduser("~/ros2_amr_ws/config/waypoints.xml"),
    os.path.expanduser("~/ros2_amr_ws/waypoints.xml"),
    os.path.join(_BASE_DIR, "../config/waypoints.xml"),
]
XML_PATH = next((p for p in DEFAULT_XML_PATHS if os.path.exists(p)), DEFAULT_XML_PATHS[0])

DEFAULT_RUNNER_PATHS = [
    os.path.expanduser("~/ros2_amr_ws/tools/run_xml_waypoints.py"),
    os.path.expanduser("~/ros2_amr_ws/run_xml_waypoints.py"),
    os.path.join(_BASE_DIR, "../tools/run_xml_waypoints.py"),
]
RUNNER_PATH = next((p for p in DEFAULT_RUNNER_PATHS if os.path.exists(p)), DEFAULT_RUNNER_PATHS[0])

DEFAULT_COORDS_PATHS = [
    os.path.expanduser("~/ros2_amr_ws/tools/navigate_to_coords.py"),
    os.path.expanduser("~/ros2_amr_ws/navigate_to_coords.py"),
    os.path.join(_BASE_DIR, "../tools/navigate_to_coords.py"),
]
COORDS_PATH = next((p for p in DEFAULT_COORDS_PATHS if os.path.exists(p)), DEFAULT_COORDS_PATHS[0])

DEFAULT_TEST_PATHS = [
    os.path.expanduser("~/ros2_amr_ws/tools/test_round_trip.py"),
    os.path.expanduser("~/ros2_amr_ws/test_round_trip.py"),
    os.path.join(_BASE_DIR, "../tools/test_round_trip.py"),
]
TEST_ROUND_TRIP_PATH = next((p for p in DEFAULT_TEST_PATHS if os.path.exists(p)), DEFAULT_TEST_PATHS[0])


def get_waypoints_dict() -> dict:
    """waypoints.xml에서 ID, 이름, 좌표 정보 파싱"""
    if not os.path.exists(XML_PATH):
        return {}
    try:
        tree = ET.parse(XML_PATH)
        root = tree.getroot()
        wps = {}
        for wp in root.findall('waypoint'):
            wp_id = wp.get('id')
            name = wp.get('name', wp_id)
            x = float(wp.find('x').text) if wp.find('x') is not None else 0.0
            y = float(wp.find('y').text) if wp.find('y') is not None else 0.0
            yaw = float(wp.find('yaw').text) if wp.find('yaw') is not None else 0.0
            wps[wp_id] = {'name': name, 'x': x, 'y': y, 'yaw': yaw}
        return wps
    except Exception as e:
        print(f"⚠️ XML 읽기 오류: {e}")
        return {}
