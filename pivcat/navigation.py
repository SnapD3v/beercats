import numpy as np
import asyncio
from movement_correct import MovementCorrector
from path_planner import *
from vision_system import VisionSystem


class NavigationController:
    def __init__(self, robot, vision: VisionSystem, config):
        self.robot = robot
        self.vision = vision
        self.corrector = MovementCorrector(self.robot.connection)
        self.config = config
        self.current_path = []
        self.current_goal = None

    async def process_frame(self, frame):
        robot_bbox = self.vision.detect_robot(frame)
        if not robot_bbox:
            return False

        # Path planning logic
        obstacle_map = self.create_obstacle_map(frame)
        start = self.calculate_scaled_coords(robot_bbox)
        path = PathPlanner(obstacle_map).plan_path(start, self.current_goal)

        if path:
            self.execute_path(path)
            return True
        return False
    import cv2

    def get_contours(self, frame, x1, y1, x2, y2, gain=0):
        # Применение медианного размытия
        blurVar = 1
        frame = cv2.medianBlur(frame, 1 + blurVar * 2)

        # Создание структурного элемента для морфологических операций
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))

        # Определение координат прямоугольника
        top_left = (x1 - gain, y1 - gain)
        bottom_right = (x2 + gain, y2 + gain)

        # контуры препятствий
        contours, _ = cv2.findContours(
            frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def hide_robot(self, frame, x1, y1, x2, y2, gain=0):
        # Применение медианного размытия
        blurVar = 1
        frame = cv2.medianBlur(frame, 1 + blurVar * 2)

        # Создание маски
        mask = cv2.inRange(frame, (10, 10, 20), (110, 96, 95))

        #

    def create_obstacle_map(self, frame,  scale_factor=4, robot_radius=10):
        contours = self.get_contours(frame, self.vision.detect_robot(frame))
        small_frame = cv2.resize(
            frame, (frame.shape[1] // scale_factor,
                    frame.shape[0] // scale_factor)
        )
        obstacle_map = np.zeros(small_frame.shape[:2], dtype=np.uint8)
        for contour in contours:
            contour = contour / scale_factor
            cv2.drawContours(obstacle_map, [contour.astype(int)], -1, 1, -1)

        # Учитываем радиус робота (расширяем препятствия, но с уменьшенным радиусом)
        kernel_size = int((robot_radius * 0.7) /
                          scale_factor)  # Уменьшаем радиус на 30%
        if kernel_size > 0:
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            obstacle_map = cv2.dilate(obstacle_map, kernel, iterations=1)

        return obstacle_map, scale_factor
