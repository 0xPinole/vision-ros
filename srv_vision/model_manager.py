"""File created to manage the AI model."""

from store_manager import Logs
from ultralytics import YOLO


class Evaluation:
    """WIP."""

    def __init__(self):
        """WIP."""
        self.model = YOLO("models/ultralytics_yolov8_model.pt")
        self.logger = Logs()

    def evaluate(self, frame) -> list[list[int]]:
        """WIP."""
        result = self.model.predict(frame)
        result = result[0]
        reduce = self.simplify(result.boxes.xywh.numpy(), result.boxes.cls.numpy())
        return reduce

    def simplify(self, boxes: list[list[float]], cls: list[float]) -> list[list[str]]:
        """Search over boxes to relate."""
        reduced_array = []
        return reduced_array
