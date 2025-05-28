import time
import socket

# Constants
STEP_COUNT = 1
ROBOT_RADIUS = 8
CORRECT_TIME = 160
COORDINATE_DIFFERENCE_FOR_CORRECT = 2
TIME_TO_1STEP_FORWARD = 450
TIME_TO_90 = 450
TIME_TO_180 = 900
DELTA_ANGLE = 0
HOST = "192.168.1.1"
PORT = 2001
RTSP_URL = "rtsp://admin:UrFU_ISIT@10.32.9.223:554/Streaming/channels/101"
LEFT_SPEED = 0x36
RIGHT_SPEED = 0x36
GO_FORWARD = b"\xff\x00\x01\x00\xff"
GO_BACK = b"\xff\x00\x02\x00\xff"
TURN_RIGHT = b"\xff\x00\x04\x00\xff"
TURN_LEFT = b"\xff\x00\x03\x00\xff"
STOP_COMMAND = b"\xff\x00\x00\x00\xff"


def do(connection, milisecounds, command):
    try:
        # Отправляем команду
        connection.sendall(command)
        sleep(milisecounds)

        # Корректировка влево
        # connection.sendall(turn_left)
        # sleep(1)

        # Команда стоп
        # connection.sendall(b"\xff\x00\x00\x00\xff")
        sleep(1)
        # connection.sendall(STOP_COMMAND)

    except socket.error as e:
        print(f"Ошибка сокета: {e}")
        connection.close()
        return False


def sleep(milisecounds):
    time.sleep(milisecounds / 1000)
