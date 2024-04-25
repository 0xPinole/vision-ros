"""Manager for camera frames getter."""

import cv2


class Camera:
    """Class owner of camera."""

    def __init__(self):
        """Get camera running."""
        try:
            self.camera = cv2.VideoCapture(0)
            if not self.camera.isOpened():
                raise ValueError("Error: Camera not detected")
        except Exception:
            self.camera = None
        except ValueError:
            self.camera = None

    def __enter__(self):
        """Logic created for ContextManager."""
        # Logic called before entry.
        pass

    def __exit__(self, exception_type, exception_val, trace):
        """Logic for close class."""
        self.release()
        return True

    def get_frame(self) -> list[list[int]]:
        """Read actual video frame."""
        for _ in range(10):
            ret, frame = self.camera.read()
            if ret:
                return frame
        return None

    def release(self):
        """Close camera after use it."""
        self.camera.release()
