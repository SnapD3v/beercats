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

# Constants
STEP_COUNT = 1
ROBOT_RADIUS = 10
CORRECT_TIME = 160
COORDINATE_DIFFERENCE_FOR_CORRECT = 2
TIME_TO_1STEP_FORWARD = 450
TIME_TO_90 = 760
TIME_TO_180 = 980
DELTA_ANGLE = 0
HOST = "192.168.1.1"
PORT = 2001
RTSP_URL = "rtsp://admin:UrFU_ISIT@10.32.9.223:554/Streaming/channels/101"
LEFT_SPEED = 0x32
RIGHT_SPEED = 0x32

# Commands
GO_FORWARD = b"\xff\x00\x01\x00\xff"
GO_BACK = b"\xff\x00\x02\x00\xff"
TURN_RIGHT = b"\xff\x00\x04\x00\xff"
TURN_LEFT = b"\xff\x00\x03\x00\xff"
STOP_COMMAND = b"\xff\x00\x00\x00\xff"


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
        self.crop_x, self.crop_y = 588, 690
        self.crop_w, self.crop_h = 1315, 230

    def capture_frame(self):
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
        self.model = YOLO(model_path)
        print(self.model.names)

    def detect_robot(self, frame):
        results = self.model.predict(frame, conf=0.3)
        annotated_frame = results[0].plot()
        cv2.imshow("Detections", annotated_frame)
        cv2.waitKey(0)
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

    def plan_path(self, frame, robot_cords, is_first):
        if is_first:
            angles, processed_angles = image_processor.process_image(
                frame, robot_cords, self.robot_radius, self.scale_factor, self.safety_margin
            )
        else:
            angles, processed_angles = image_processor.secound_process_image(
                frame, robot_cords, self.robot_radius, self.scale_factor, self.safety_margin
            )
        return self.convert_angles_to_directions(angles)

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
        self.send_command(STOP_COMMAND)
        time.sleep(1)

    def execute_movement(self, current_vector, next_direction):
        if abs(next_direction - current_vector) == 2:
            print(
                f"180° turn from {self.direction_name(current_vector)} to {self.direction_name(next_direction)}")
            if (next_direction == 1 and current_vector == -1) or (next_direction == 0 and current_vector == -2):
                self.do(TIME_TO_90, TURN_RIGHT)
                time.sleep(0.5)
                self.do(TIME_TO_90, TURN_RIGHT)
                return (current_vector + 2) % 4
            elif (next_direction == -1 and current_vector == 1) or (next_direction == -2 and current_vector == 0):
                self.do(TIME_TO_90, TURN_LEFT)
                time.sleep(0.5)
                self.do(TIME_TO_90, TURN_LEFT)
                return (current_vector - 2) % 4
        elif next_direction == 1 and current_vector == -2:
            self.do(TIME_TO_1STEP_FORWARD, GO_FORWARD)
            time.sleep(0.5)
            self.do(TIME_TO_90 + DELTA_ANGLE, TURN_LEFT)
            print("Turn LEFT")
            return (current_vector + 3) % 4
        elif next_direction == -2 and current_vector == 1:
            self.do(TIME_TO_1STEP_FORWARD, GO_FORWARD)
            time.sleep(0.5)
            self.do(TIME_TO_90, TURN_RIGHT)
            print("Turn RIGHT")
            return (current_vector - 3) % 4
        elif next_direction > current_vector:
            self.do(TIME_TO_1STEP_FORWARD, GO_FORWARD)
            time.sleep(0.5)
            self.do(TIME_TO_90, TURN_RIGHT)
            print("Turn RIGHT")
            return (current_vector + 1) % 4
        elif next_direction < current_vector:
            self.do(TIME_TO_1STEP_FORWARD, GO_FORWARD)
            time.sleep(0.5)
            self.do(TIME_TO_90 + DELTA_ANGLE, TURN_LEFT)
            print("Turn LEFT")
            return (current_vector - 1) % 4
        else:
            self.do(TIME_TO_1STEP_FORWARD, GO_FORWARD)
            print("FORWARD")
            return current_vector

    def direction_name(self, vector):
        directions = {0: "RIGHT", 1: "DOWN", -1: "UP", -2: "LEFT"}
        return directions.get(vector, "UNKNOWN")


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
        self.prev_robot_center = None
        self.step_count = STEP_COUNT

    def run(self):
        print("\033[32m", end="")
        tprint("Hello   world")
        print("\033[0m")
        print("\033[31m", end="")
        tprint("DIR   I   Love   You")
        print("\033[0m")
        print("Connecting to camera")
        print("Connected")

        i = 0
        while True:
            i += 1
            print(f"Step: {i}")
            frame = self.camera.capture_frame()
            if frame is None:
                cv2.imshow("Cropped Frame", frame)
                cv2.waitKey(0)
                continue

            robot_center, robot_cords = self.robot_detector.detect_robot(frame)
            if robot_center is None or robot_cords is None:
                continue
            print(f"Robot center: {robot_center}")

            if self.angles is None:
                try:
                    self.angles = self.path_planner.plan_path(
                        frame, robot_cords, is_first=(i == 1))
                    print(f"Planned directions: {self.angles}")
                except PathError:
                    continue

            steps_taken = 0
            while steps_taken < self.step_count and self.angles:
                if len(self.angles) > 1 and self.angles[1] != 0:
                    self.angles = self.angles[1:]
                    print("PREDICT TURN!")
                next_direction = self.angles.pop(0)
                print(
                    f"Robot oriented: {self.robot_controller.direction_name(self.vector)}")
                print(
                    f"Next step: {self.robot_controller.direction_name(next_direction)}")
                self.vector = self.robot_controller.execute_movement(
                    self.vector, next_direction)

                if self.vector == next_direction:  # Only correct if going straight
                    new_frame = self.camera.capture_frame()
                    new_robot_center, _ = self.robot_detector.detect_robot(
                        new_frame)
                    if new_robot_center:
                        self.movement_corrector.correct_direction(
                            self.vector, new_robot_center, self.prev_robot_center, self.robot_controller
                        )
                        true_vector = self.movement_corrector.find_true_vector(
                            self.prev_robot_center, new_robot_center
                        )
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

        self.robot_controller.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    navigator = Navigator()
    navigator.run()
