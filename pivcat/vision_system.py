import cv2
import numpy as np
from ultralytics import YOLO


class VisionSystem:
    def __init__(self, model_path):
        self.model = YOLO(model=model_path)
        self.calibration = {
            'matrix': np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=np.float32),
            'dist_coeffs': np.zeros((4, 1))
        }

    def remove_distortion(self, frame):
        h, w = frame.shape[:2]
        new_cam_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.calibration['matrix'],
            self.calibration['dist_coeffs'],
            (w, h), 1, (w, h)
        )
        return cv2.undistort(frame, self.calibration['matrix'],
                             self.calibration['dist_coeffs'], None, new_cam_matrix)

    def detect_robot(self, frame):
        results = self.model(frame)
        for result in results:
            for box in result.boxes:
                if box.cls[0].item() == 0:  # Assuming class 0 is robot
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    return (int(x1), int(y1), int(x2), int(y2))
        return None
