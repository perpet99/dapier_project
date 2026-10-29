import cv2
import numpy as np

# OpenCV 4.x ArUco dictionary for AprilTag 36h11
dict_36h11 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

# Method 1: drawMarker
if hasattr(cv2.aruco, 'drawMarker'):
    tag_img = cv2.aruco.drawMarker(dict_36h11, 1, 400)
elif hasattr(cv2.aruco, 'generateImageMarker'):
    tag_img = cv2.aruco.generateImageMarker(dict_36h11, 1, 400)
else:
    # fallback
    tag_img = None

if tag_img is not None:
    # Add large clean white border (Quiet Zone)
    bordered = cv2.copyMakeBorder(tag_img, 80, 80, 80, 80, cv2.BORDER_CONSTANT, value=255)
    out_path = '/home/juyeon/ros2_amr_ws/src/amr_bringup/models/apriltag_marker/materials/textures/tag36h11_1.png'
    cv2.imwrite(out_path, bordered)
    print(f"Successfully generated authentic AprilTag 36h11 #1 texture: {bordered.shape}")

    # Verify detection
    detector = cv2.aruco.ArucoDetector(dict_36h11) if hasattr(cv2.aruco, 'ArucoDetector') else None
    if detector:
        corners, ids, _ = detector.detectMarkers(bordered)
        print("Detection test result IDs:", ids)
    elif hasattr(cv2.aruco, 'detectMarkers'):
        corners, ids, _ = cv2.aruco.detectMarkers(bordered, dict_36h11)
        print("Detection test result IDs:", ids)
else:
    print("Failed to find drawMarker/generateImageMarker")
