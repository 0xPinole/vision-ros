import cv2
import numpy as np
import yaml

# Checkerboard dimensions
CHECKERBOARD = (7, 7)

# Termination criteria for corner sub-pixel accuracy
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points based on checkerboard dimensions
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all images
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane

cap = cv2.VideoCapture(2)

while len(objpoints) < 10:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret)

    cv2.imshow('Calibration', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# Camera calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Save calibration data to YAML file
data = {
    'camera_name': 'usb_cam',
    'image_width': frame.shape[1],
    'image_height': frame.shape[0],
    'camera_matrix': {'rows': 3, 'cols': 3, 'data': mtx.flatten().tolist()},
    'distortion_coefficients': {'rows': 1, 'cols': 5, 'data': dist.flatten().tolist()},
    'rectification_matrix': {'rows': 3, 'cols': 3, 'data': np.eye(3).flatten().tolist()},
    'projection_matrix': {'rows': 3, 'cols': 4, 'data': np.hstack((mtx, np.zeros((3, 1)))).flatten().tolist()}
}

with open("calibration.yaml", "w") as f:
    yaml.dump(data, f)
