"""File created to manage the AI model."""

# import numpy as np
# from sklearn.cluster import MeanShift
from ultralytics import YOLO


class Evaluation:
    """WIP."""

    def __init__(self):
        """WIP."""
        self.model = YOLO("models/ultralytics_yolov8_model.pt")
        # self.mean_shift = MeanShift(bandwidth=None, bin_seeding=True)

    def evaluate(self, frame) -> list[list[int]]:
        """WIP."""
        result = self.model.predict(frame)
        result = result[0]
        reduce = self.simplify(result.boxes.xywh.numpy(), result.boxes.cls.numpy())
        return reduce

    def simplify(self, boxes: list[list[float]], cls: list[float]) -> list[list[str]]:
        """Search over boxes to relate."""
        # cluster_array = np.reshape(boxes, (-1, 1))
        # self.mean_shift.fit(cluster_array)
        # cluster_centers = ms.cluster_centers_
        # labels = self.mean_shift.labels_

        # labels_unique = np.unique(labels)
        # n_clusters_ = len(labels_unique)
        reduced_array = []
        return reduced_array
