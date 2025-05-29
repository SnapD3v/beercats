import cv2
import numpy as np
from ultralytics import YOLO
import time
import socket
import asyncio
from art import tprint
import image_processor
from movement_correct import correct_direction, find_true_vector
from path_error import PathError
from config import *

# # Constants
# STEP_COUNT = 1
# ROBOT_RADIUS = 10
# CORRECT_TIME = 160
# COORDINATE_DIFFERENCE_FOR_CORRECT = 2
# TIME_TO_1STEP_FORWARD = 450//10
# TIME_TO_90 = 760//10
# TIME_TO_180 = 980//10
# DELTA_ANGLE = 0
# HOST = "192.168.1.1"
# PORT = 2001
# RTSP_URL = "rtsp://admin:UrFU_ISIT@10.32.9.223:554/Streaming/channels/101"
# LEFT_SPEED = 0x36
# RIGHT_SPEED = 0x36

# Commands


class Camera:
    def __init__(self, rtsp_url):
        self.rtsp_url = rtsp_url
        self.camera_matrix = np.array([
            [5.47045703e+02, 0.00000000e+00, 1.27382808e+03],
            [0.00000000e+00, 5.28615087e+02, 9.63441154e+02],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ])
        self.dist_coeffs = np.array(
            [-0.31517132, 0.10796146, -0.0019241, 0.00132282, -0.01692133])
        self.crop_x, self.crop_y = 500, 750
        self.crop_w, self.crop_h = 1315, 500

    def capture_frame(self):
        """
        Capture a frame from the camera.

        Returns:
            A frame as a 3D numpy array, or None if capture fails.
        """
        cap = cv2.VideoCapture(self.rtsp_url)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        for _ in range(2):
            cap.grab()
        ret, frame = cap.read()
        cap.release()
        if not ret:
            return None
        undist = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        return undist[self.crop_y:self.crop_y + self.crop_h, self.crop_x:self.crop_x + self.crop_w]


class RobotDetector:
    def __init__(self, model_path="best.pt"):
        """
        Initialize the robot detector.

        Args:
            model_path (str, optional): Path to the YOLO model. Defaults to "best.pt".
        """
        self.model = YOLO(model_path)

    def detect_robot(self, frame):
        """
        Detect the robot in the frame.

        Args:
            frame: The frame to detect the robot in.

        Returns:
            A tuple of two elements. The first element is the center of the robot as a tuple of two integers (x, y).
            The second element is the bounding box of the robot as a tuple of four integers (x1, y1, x2, y2).
            If the robot is not detected, both elements are None.
        """
        results = self.model.predict(frame, conf=0.3)
        for r in results:
            for box in r.boxes:
                if box.cls[0].item() == 0:  # Assuming class 0 is the robot
                    x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                    center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                    return center, (x1, y1, x2, y2)
        return None, None


class PathPlanner:
    def __init__(self, robot_radius=ROBOT_RADIUS, scale_factor=5, safety_margin=1):
        self.robot_radius = robot_radius
        self.scale_factor = scale_factor
        self.safety_margin = safety_margin

    async def plan_path(self, frame, robot_cords, is_first):
        """
        Plan the path for the robot.

        Args:
            frame: The frame to detect the robot and obstacles in.
            robot_cords: The coordinates of the robot as a tuple of four integers (x1, y1, x2, y2).
            is_first: A boolean indicating whether this is the first frame.

        Returns:
            A list of directions as integers. The directions are one of -2 (left), -1 (forward-left), 0 (forward), 1 (forward-right), or 2 (right).
        """
        if frame is None or robot_cords is None:
            raise ValueError("frame and robot_cords cannot be null")
        try:
            if is_first:
                angles, processed_angles = await image_processor.process_image(
                    frame, robot_cords, self.robot_radius, self.scale_factor, self.safety_margin
                )
            else:
                angles, processed_angles = await image_processor.secound_process_image(
                    frame, robot_cords, self.robot_radius, self.scale_factor, self.safety_margin
                )
            if angles is None:
                raise PathError("No path found")
            return self.convert_angles_to_directions(angles)
        except Exception as e:
            print(f"Error while planning path: {e}")
            raise PathError("Error while planning path") from e

    def convert_angles_to_directions(self, angles):
        directions = []
        for angle in angles:
            if angle == -np.pi:
                directions.append(-2)
            elif angle == -np.pi / 2:
                directions.append(-1)
            elif angle == np.pi / 2:
                directions.append(1)
            elif angle == 0:
                directions.append(0)
        return directions


class RobotController:
    def __init__(self, host=HOST, port=PORT):
        self.host = host
        self.port = port
        self.connection = self.connect()
        self.set_speed(LEFT_SPEED, RIGHT_SPEED)

    def connect(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f"Connecting to {self.host}:{self.port}")
        s.connect((self.host, self.port))
        return s

    def close(self):
        if self.connection:
            self.connection.close()
            print("Connection closed")

    def send_command(self, command):
        try:
            self.connection.sendall(command)
            time.sleep(1)
        except socket.error as e:
            print(f"Socket error: {e}")
            self.close()

    def set_speed(self, left_speed, right_speed):
        self.send_command(b'\xff\x02\x01' + bytes([left_speed]) + b'\xff')
        self.send_command(b'\xff\x02\x02' + bytes([right_speed]) + b'\xff')

    def do(self, milliseconds, command):
        self.send_command(command)
        time.sleep(milliseconds / 1000)
        # time.sleep(1)
        self.send_command(STOP_COMMAND)

    def execute_movement(self, current_vector, next_direction):
        """
    Выполняет движение робота из текущего направления в следующее.

    Параметры:
        current_vector: текущее направление робота (-2: влево, -1: вверх, 0: вправо, 1: вниз)
        next_direction: следующее целевое направление

    Возвращает:
        Новое текущее направление после выполнения движения
    """

        # 1. Проверка на разворот на 180 градусов
        if abs(next_direction - current_vector) == 2:
            print(
                f"180° turn from {self.direction_name(current_vector)} to {self.direction_name(next_direction)}")

            # Случай 1: Разворот через два поворота направо
            if (next_direction == 1 and current_vector == -1) or (next_direction == 0 and current_vector == -2):
                self.do(TIME_TO_90, TURN_RIGHT)  # Первый поворот направо
                time.sleep(0.5)  # Пауза между поворотами
                self.do(TIME_TO_90, TURN_RIGHT)  # Второй поворот направо
                return (current_vector + 2) % 4  # Обновляем направление

            # Случай 2: Разворот через два поворота налево
            elif (next_direction == -1 and current_vector == 1) or (next_direction == -2 and current_vector == 0):
                self.do(TIME_TO_90, TURN_LEFT)  # Первый поворот налево
                time.sleep(0.5)  # Пауза между поворотами
                self.do(TIME_TO_90, TURN_LEFT)  # Второй поворот налево
                return (current_vector - 2) % 4  # Обновляем направление

        # 2. Специальные случаи для переходов через ноль (270° повороты)

        # Случай 3: Из "влево" (-2) в "вниз" (1) - поворот налево на 270°
        elif next_direction == 1 and current_vector == -2:
            # Сначала небольшой движение вперед
            self.do(TIME_TO_1STEP_FORWARD, GO_FORWARD)
            time.sleep(0.5)
            # Поворот налево с коррекцией угла (DELTA_ANGLE)
            self.do(TIME_TO_90 + DELTA_ANGLE, TURN_LEFT)
            print("Turn LEFT")
            return (current_vector + 3) % 4  # Обновляем направление

        # Случай 4: Из "вниз" (1) в "влево" (-2) - поворот направо на 270°
        elif next_direction == -2 and current_vector == 1:
            # Сначала небольшой движение вперед
            self.do(TIME_TO_1STEP_FORWARD, GO_FORWARD)
            time.sleep(0.5)
            self.do(TIME_TO_90, TURN_RIGHT)  # Поворот направо
            print("Turn RIGHT")
            return (current_vector - 3) % 4  # Обновляем направление

        # 3. Стандартные повороты на 90 градусов

        # Случай 5: Поворот направо (next_direction > current_vector)
        elif next_direction > current_vector:
            # Сначала небольшой движение вперед
            self.do(TIME_TO_1STEP_FORWARD, GO_FORWARD)
            time.sleep(0.5)
            self.do(TIME_TO_90, TURN_RIGHT)  # Поворот направо
            print("Turn RIGHT")
            return (current_vector + 1) % 4  # Обновляем направление

        # Случай 6: Поворот налево (next_direction < current_vector)
        elif next_direction < current_vector:
            # Сначала небольшой движение вперед
            self.do(TIME_TO_1STEP_FORWARD, GO_FORWARD)
            time.sleep(0.5)
            # Поворот налево с коррекцией угла (DELTA_ANGLE)
            self.do(TIME_TO_90 + DELTA_ANGLE, TURN_LEFT)
            print("Turn LEFT")
            return (current_vector - 1) % 4  # Обновляем направление

        # 4. Движение прямо (направления совпадают)
        else:
            # Просто двигаемся вперед
            self.do(TIME_TO_1STEP_FORWARD, GO_FORWARD)
            print("FORWARD")
            return current_vector  # Направление не меняется


class MovementCorrector:
    @staticmethod
    def correct_direction(vector, robot_center, prev_robot_center, controller):
        correct_direction(vector, robot_center,
                          prev_robot_center, controller.connection)

    @staticmethod
    def find_true_vector(prev_robot_center, robot_center):
        return find_true_vector(prev_robot_center, robot_center)


class Navigator:
    def __init__(self):
        self.camera = Camera(RTSP_URL)
        self.robot_detector = RobotDetector()
        self.path_planner = PathPlanner()
        self.robot_controller = RobotController()
        self.movement_corrector = MovementCorrector()
        self.angles = None
        self.vector = 0
        self.prev_robot_center = [0, 0]
        self.step_count = STEP_COUNT

    async def run(self):
        print("\033[32m", end="")
        tprint("Hello   world")
        print("\033[0m")
        print("\033[31m", end="")
        tprint("huy   huy")
        print("\033[0m")
        print("Connecting to camera")
        print("Connected")

        i = 0
        while True:
            i += 1
            print(f"Step: {i}")
            frame = self.camera.capture_frame()
            print('снимок')
            if frame is None:
                continue

            robot_center, robot_cords = self.robot_detector.detect_robot(frame)
            if robot_center is None or robot_cords is None:
                continue
            print(f"Robot center: {robot_center}")

            if self.angles is None:
                try:
                    self.angles = await self.path_planner.plan_path(
                        frame, robot_cords, is_first=(i == 1)
                    )
                    print(f"Planned directions: {self.angles}")
                except PathError:
                    continue

            steps_taken = 0
            while steps_taken < self.step_count and self.angles:
                self.angles = self.angles[1:]
                next_direction = self.angles.pop(0)
                print(
                    f"Robot повёрнут: {self.robot_controller.direction_name(self.vector)}")
                print(
                    f"Сейчас делает: {self.robot_controller.direction_name(next_direction)}")
                self.vector = self.robot_controller.execute_movement(
                    self.vector, next_direction)

                if self.vector == next_direction:
                    new_frame = self.camera.capture_frame()
                    new_robot_center, _ = self.robot_detector.detect_robot(
                        new_frame)
                    print('___поиск робота')
                    if new_robot_center:
                        self.movement_corrector.correct_direction(
                            self.vector, new_robot_center, self.prev_robot_center, self.robot_controller
                        )
                        print('___correct direction')
                        true_vector = self.movement_corrector.find_true_vector(
                            self.prev_robot_center, new_robot_center
                        )
                        print('определение вектора направления')
                        if true_vector is not None and true_vector != self.vector:
                            print(f"Vector changed from {self.robot_controller.direction_name(self.vector)} "
                                  f"to {self.robot_controller.direction_name(true_vector)}")
                            self.vector = true_vector
                self.prev_robot_center = new_robot_center

                steps_taken += 1
                if steps_taken == self.step_count:
                    self.angles = None
                    break

            if not self.angles:
                self.angles = None


if __name__ == "__main__":
    navigator = Navigator()
    asyncio.run(navigator.run())
