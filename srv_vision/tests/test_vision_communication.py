from ultralytics import YOLO
import json
import cv2
import requests

headers = {"Content-Type": "application/json"}
with open("/home/pinole/ros2_ws/src/srv_vision/srv_vision/db/products.json", "r") as fg:
    products = json.load(fg)

def get_product_by_hash_vision(query: str) -> dict[str, any]:
    """WIP."""
    uuid = products["vision_hash"].get(query)
    if uuid is None:
        return None
    product = products["products"].get(uuid)
    return product

def format_product(vision_hash: str, stock: str, aisle: str, rack: str):
    """WIP."""
    product_info = get_product_by_hash_vision(vision_hash)
    if product_info is not None:
        json_send = {
            "PLU": product_info["plu"],
            "Product": product_info["name"],
            "Type": product_info["variety"],
            "Aisle": aisle,
            "Rack": rack,
            "AmountS": stock,
            "AmountF": "0",
            "Supply": False,
        }

    return json_send

def post(sender: dict[str, any]) -> str:
    """WIP."""
    try:
        response = requests.post(
            "http://0.0.0.0:9000/", # base64.b64decode(self.endpoint_s1 + self.endpoint_s2).decode("utf-8"),
            json=sender,
            headers=headers,
        )
        response.raise_for_status()
    except requests.exceptions.RequestException as e:
        if hasattr(e, "response"):
            return f"Error: code {e.response.status_code} received"
        return "Error: Unreachable server"
    return f"Ok: code {response.status_code} received"

camera = cv2.VideoCapture(0)

model = YOLO("/home/pinole/ros2_ws/src/srv_vision/srv_vision/models/ultralytics_yolov8_model.pt")

detections = {}

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

        for class_i in classes_bx:
            vision_hash = model.names[class_i]
            if detections.get(vision_hash) is None:
                detections[vision_hash] = 0
            detections[vision_hash] += 1

for vision_hash, stock in list(detections.items()):
    json_formated = format_product(vision_hash, stock, 1, 1)
    post(json_formated)
