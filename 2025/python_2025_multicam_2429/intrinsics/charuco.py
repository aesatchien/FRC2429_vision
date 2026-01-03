#CHARUCO_BOARD = aruco.CharucoBoard((9, 6), 0.015, 0.011, aruco.getPredefinedDictionary(aruco.DICT_4X4_250))

import cv2
import numpy as np

# 1. Setup Board (Using the new DetectorParameters and Dictionary)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
board = cv2.aruco.CharucoBoard((9, 6), 0.015, 0.011, dictionary)
# detector = cv2.aruco.CharucoDetector(board)

# 2. Marker Detection Parameters
marker_params = cv2.aruco.DetectorParameters()
# This is where the attribute actually lives:
marker_params.tryExtendMarkers = True
marker_params.minMarkerPerimeterRate = 0.01

# 3. Charuco Parameters (Usually okay with defaults)
charuco_params = cv2.aruco.CharucoParameters()

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

all_charuco_corners = []
all_charuco_ids = []
image_size = None

print("Press 's' to capture, 'q' to calibrate")

while True:
    ret, frame = cap.read()
    if not ret: break

    # 2. Use the new Detector object (This replaces detectMarkers/interpolate)
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(frame)

    if charuco_ids is not None:
        cv2.aruco.drawDetectedCornersCharuco(frame, charuco_corners, charuco_ids)

    cv2.imshow('640x360 Calibration', frame)
    key = cv2.waitKey(1)
    if key == ord('s') and charuco_ids is not None:
        all_charuco_corners.append(charuco_corners)
        all_charuco_ids.append(charuco_ids)
        image_size = (frame.shape[1], frame.shape[0])
        print(f"Captured {len(all_charuco_corners)} frames")
    elif key == ord('q'):
        break

# 3. Calibrate using the accumulated data
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
    charuco_corners=all_charuco_corners,
    charuco_ids=all_charuco_ids,
    board=board,
    imageSize=image_size,
    cameraMatrix=None,
    distCoeffs=None
)

print("\n--- Calibration Results (640x360) ---")
print(camera_matrix)
cap.release()
cv2.destroyAllWindows()