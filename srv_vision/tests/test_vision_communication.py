from ultralytics import YOLO
import cv2

camera = cv2.VideoCapture(0)

model = YOLO("/home/pinole/ros2_ws/src/srv_vision/srv_vision/models/ultralytics_yolov8_model.pt")


while True:
    _, frame = camera.read()

    cv2.imshow("test", frame)

    key = cv2.waitKey(0)
    if key == 27:
        break
    elif key == 32:
        result = model.predict(frame)
        result = result[0]
        bxs = result.boxes.cpu()
        coords_bx = bxs.xywh.numpy()
        classes_bx = bxs.cls.numpy()

        print(coords_bx)
        print(classes_bx)
