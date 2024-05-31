import cv2
import numpy as np
import yaml

# Load calibration parameters from YAML file
with open("./fisheye_calibration.yaml", "r") as f:
    calib_data = yaml.safe_load(f)

# Extract the parameters
K = np.array(calib_data['camera_matrix']['data']).reshape((3, 3)).astype(np.float64)
dist_coeffs = calib_data['distortion_coefficients']['data']

# Handle both 4 and 5 distortion coefficients, and convert to the correct type
if len(dist_coeffs) == 4:
    D = np.array(dist_coeffs).reshape((4, 1)).astype(np.float64)
elif len(dist_coeffs) == 5:
    D = np.array(dist_coeffs).reshape((5, 1)).astype(np.float64)
else:
    raise ValueError("Unexpected number of distortion coefficients: {}".format(len(dist_coeffs)))

image_width = calib_data['image_width']
image_height = calib_data['image_height']

# Open the camera
cap = cv2.VideoCapture(2)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab a frame")
        break

    h, w = frame.shape[:2]

    # Scale the camera matrix if needed
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D[:4], (w, h), np.eye(3), balance=1)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D[:4], np.eye(3), new_K, (w, h), cv2.CV_16SC2)
    undistorted_img = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    # Display the original and undistorted images
    cv2.imshow('Original', frame)
    cv2.imshow('Undistorted', undistorted_img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
