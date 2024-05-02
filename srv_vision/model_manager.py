"""File created to manage the AI model."""

import numpy as np
from sklearn.cluster import MeanShift
from ultralytics import YOLO


class Evaluation:
    """WIP."""

    def __init__(self):
        """WIP."""
        self.model = YOLO("models/ultralytics_yolov8_model.pt")
        self.mean_shift = MeanShift(bandwidth=None, bin_seeding=True)

    def evaluate(self, frame) -> list[list[int]]:
        """WIP."""
        result = self.model.predict(frame)
        result = result[0]
        reduce = self.simplify(result.boxes.xywh.numpy(), result.boxes.cls.numpy())
        return reduce

    def simplify(
        self, boxes: list[list[float]], cls_prods: list[float]
    ) -> list[list[str]]:
        """Search over boxes to relate."""
        # x_lines = boxes[:,1] + boxes[:,2]
        # x_lines = np.reshape(x_lines, (-1, 1))
        # self.mean_shift.fit(cluster_array)
        # cluster_centers = ms.cluster_centers_
        # labels = self.mean_shift.labels_
        #
        labels_unique = np.unique(cls_prods)
        # n_clusters_ = len(labels_unique)
        # reduced_array = []
        # cols = []
        #
        # for label in labels_unique:
        #
        #   if section.get(label) is None:
        #       section[label] = []
        #
        #   cluster_boxes[label].append([boxes[index], cls[index]])

        # reduced_array = []
        reduced_array = {}

        for label in labels_unique:
            reduced_array[label] = len(cls_prods[cls_prods == label])

        return reduced_array
