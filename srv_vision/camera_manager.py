"""Manager for camera frames getter."""

import cv2

from srv_vision.camera_configuration import CamConfig


class Camera(CamConfig):
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

        if self.camera is not None:
            self.flags_setter()

    def __enter__(self):
        """Logic created for ContextManager."""
        # Logic called before entry.
        pass

    def __exit__(self, exception_type, exception_val, trace):
        """Logic for close class."""
        self.release()
        return True

    def flags_setter(self):
        """Set flags to self.camera from the configuration."""
        for flag, value in self.video_flags:
            self.camera.set(flag, value)

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
