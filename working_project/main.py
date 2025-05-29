import cv2
import numpy as np
from ultralytics import YOLO
import time
import socket
import numpy as np
from art import *
from config import *


import image_processor
from movement_correct import correct_direction, find_true_vector

from path_error import PathError

import asyncio

RED = "\033[31m"
RESET = "\033[0m"
print(RESET)

# Моделька
model = YOLO('best.pt')
# Камера
rtsp_url = "rtsp://admin:UrFU_ISIT@10.32.9.223:554/Streaming/channels/101"
# Подключимся
print("Подключаюсь к камере")
cap = cv2.VideoCapture(rtsp_url)
print("Подключено")

# region Устанавливаем параметры камеры
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Минимальный размер буфера
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(
    *'MJPG'))  # Используем MJPEG
cap.set(cv2.CAP_PROP_FPS, 30)  # Устанавливаем частоту кадров
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"Параметры камеры: FPS={30}, Разрешение={width}x{height}")
# endregion

# Ширина, высота кадра
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Матрица камеры и коэффициенты искажения~
camera_matrix = np.array([[5.47045703e+02, 0.00000000e+00, 1.27382808e+03],
                          [0.00000000e+00, 5.28615087e+02, 9.63441154e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist_coefs = np.array(
    [-0.31517132, 0.10796146, -0.0019241, 0.00132282, -0.01692133])

# Получаем оптимальную новую матрицу камеры и область обрезки
new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coefs, (frame_width, frame_height), 1,
                                                       (frame_width, frame_height))

print(model.names)

# region movement
host = "192.168.1.1"
port = 2001


def vector_dir(vector):
    if vector == 1:
        return ("вниз")
    if vector == 0:
        return ("вправо")
    if vector == -1:
        return ("вверх")
    if vector == -2:
        return ("влево")


def send_command(command):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f"Соединение с {host}:{port}")
        s.connect((host, port))
        print(f"Отправка команды: {command}")
        s.sendall(command)
        time.sleep(1)

        return True
    except socket.error as e:
        print(f"Ошибка сокета: {e}")
        return False
    finally:
        s.close()
        print("Соединение закрыто")


def connect_to_robot():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Создаем сокет

    print(f"Соединение с {host}:{port}")

    # Устанавливаем соединение
    s.connect((host, port))

    return s


def close_connection(s):
    s.close()
    print("Соединение закрыто")


def set_speed(left_speed, right_speed):
    send_command(b'\xff\x02\x01' + bytes([left_speed]) + b'\xff')
    send_command(b'\xff\x02\x02' + bytes([right_speed]) + b'\xff')

# Сделать команду по времени


def do(connection, milisecounds, command):
    try:
        connection.sendall(command)
        sleep(milisecounds)
        sleep(1)
        connection.sendall(stop_command)

    except socket.error as e:
        print(f"Ошибка сокета: {e}")
        connection.close()
        return False


def sleep(milisecounds):
    time.sleep(milisecounds / 1000)


connection = connect_to_robot()
set_speed(left_speed, right_speed)

i = 0
angles = None
prev_robot_center = None
while True:
    cap = cv2.VideoCapture(rtsp_url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    i += 1
    print(f"Это шаг: {i} ?")

    for _ in range(2):
        cap.grab()

    ret, frame = cap.read()

    cap.release()

    undist = cv2.undistort(frame, camera_matrix,
                           dist_coefs, None, new_camera_matrix)

    resized_frame = undist[crop_y:crop_y + crop_h, crop_x:crop_x + crop_w]

    results = model.predict(resized_frame, conf=0.3)

    robot_center = None
    x1, y1, x2, y2 = None, None, None, None
    mask = cv2.inRange(resized_frame, (10, 10, 20), (110, 96, 95))

    for r in results:
        boxes = r.boxes
        for box in boxes:
            if box.cls[0].item() == 0:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                robot_center_x = int((x1 + x2) / 2)
                robot_center_y = int((y1 + y2) / 2)
                print(
                    f"Координаты центра робота: x={robot_center_x}, y={robot_center_y}")

    if x1:
        print(f"координаты робота: {x1}, {y1}, {x2}, {y2}")

    annotated_frame = results[-1].plot()

    print(F"Поиск i, {i}")
    if angles == None and i == 1:
        if x1:
            try:
                angles, processed_angles = asyncio.run(image_processor.process_image(
                    resized_frame,
                    robot_radius=ROBOT_RADIUS,
                    scale_factor=5,  # SCALE_FACTOR,
                    robot_cords=(int(x1), int(y1), int(x2), int(y2)),
                    safety_margin=0.5
                ))
            except PathError:
                continue

            print(processed_angles)
            print(angles)
            new_angles = []
            for angle in angles:
                if angle == -np.pi:
                    new_angles.append(-2)
                elif angle == -np.pi/2:
                    new_angles.append(-1)
                elif angle == np.pi/2:
                    new_angles.append(1)
                elif angle == 0:
                    new_angles.append(0)
            print(new_angles)

    elif angles == None and i > 1:
        if x1:
            try:
                angles, processed_angles = asyncio.run(image_processor.secound_process_image(
                    resized_frame,
                    robot_radius=ROBOT_RADIUS,
                    scale_factor=5,
                    robot_cords=(int(x1), int(y1), int(x2), int(y2)),
                ))
            except PathError:
                continue

            print(processed_angles)
            print(angles)
            new_angles = []
            for angle in angles:
                if angle == -np.pi:
                    new_angles.append(-2)
                elif angle == -np.pi/2:
                    new_angles.append(-1)
                elif angle == np.pi/2:
                    new_angles.append(1)
                elif angle == 0:
                    new_angles.append(0)
            print(new_angles)

    else:
        c = 0
        if not new_angles:
            print(RESET)
            break
        if new_angles[1] != 0:
            new_angles = new_angles[1:]
            print("turn_predict")

        for new_angle in new_angles:
            print(f"направление - {vector_dir(vector)}")
            print(f"след шаг - {vector_dir(new_angle)}")
            print(f"текущий вектор - {vector}")
            if abs(new_angle - vector) == 2:
                if new_angle == 1 and vector == -1:
                    do(connection, time_to_90, turn_right)
                    sleep(500)
                    do(connection, time_to_90, turn_right)

                    vector += 2
                if new_angle == -1 and vector == 1:
                    do(connection, time_to_90, turn_left)
                    sleep(500)
                    do(connection, time_to_90, turn_left)

                    vector -= 2
                if new_angle == 0 and vector == -2:
                    do(connection, time_to_90, turn_right)
                    sleep(500)
                    do(connection, time_to_90, turn_right)

                    vector += 2
                if new_angle == -2 and vector == 0:
                    do(connection, time_to_90, turn_left)
                    sleep(500)
                    do(connection, time_to_90, turn_left)

                    vector -= 2
            elif new_angle == 1 and vector == -2:
                do(connection, time_to_1step_forawrd, go_forward)
                sleep(500)
                do(connection, time_to_90+DELTA_ANGLE, turn_left)
                print("_налево")
                vector += 3
            elif vector == 1 and new_angle == -2:
                do(connection, time_to_1step_forawrd, go_forward)
                sleep(500)
                do(connection, time_to_90, turn_right)
                print("_направо")
                vector -= 3

            elif new_angle > vector:
                do(connection, time_to_1step_forawrd, go_forward)
                sleep(500)
                do(connection, time_to_90, turn_right)
                print("_направо")
                vector += 1

            elif new_angle < vector:
                do(connection, time_to_1step_forawrd, go_forward)
                sleep(500)
                do(connection, time_to_90+DELTA_ANGLE, turn_left)
                print("_направо")
                vector -= 1

            else:
                do(connection, time_to_1step_forawrd, go_forward)
                print("_вперед")
                robot_center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                robot_center_x = robot_center[0]
                robot_center_y = robot_center[1]

                if prev_robot_center:
                    prev_robot_center_x = prev_robot_center[0]
                    prev_robot_center_y = prev_robot_center[1]

                    correct_direction(vector, robot_center,
                                      prev_robot_center, connection)
                    find_true_vector(prev_robot_center, robot_center)

                    print(
                        f"Настоящий вектор - {vector_dir(true_vector)}")

                    if true_vector != None:
                        if true_vector != vector:
                            print("----------------------------------------")
                            print(
                                f"Поменял вектор {vector_dir(vector)} на {vector_dir(true_vector)}")
                            print("----------------------------------------")

                            vector = true_vector
                prev_robot_center = robot_center
            c += 1
            if c == STEP_COUNT:
                angles = None
                break


close_connection(connection)
cap.release()
cv2.destroyAllWindows()
