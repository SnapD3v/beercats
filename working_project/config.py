import time
import socket

true_vector = None
go_forward = b"\xff\x00\x01\x00\xff"
go_back = b"\xff\x00\x02\x00\xff"
turn_right = b"\xff\x00\x04\x00\xff"
turn_left = b"\xff\x00\x03\x00\xff"
STEP_COUNT=1
ROBOT_RADIUS = 10
CORRECT_TIME = 160
COORDINATE_DIFFERENCE_FOR_CORRECT = 2
time_to_1step_forawrd = 450
stop_command = b"\xff\x00\x00\x00\xff"


def do(connection, milisecounds, command):
    try:
        # Отправляем команду
        connection.sendall(command)
        sleep(milisecounds)

        #Корректировка влево
        # connection.sendall(turn_left)
        # sleep(1)

        #Команда стоп
        # connection.sendall(b"\xff\x00\x00\x00\xff")
        sleep(1)
        connection.sendall(stop_command)

        
    except socket.error as e:
        print(f"Ошибка сокета: {e}")
        connection.close()
        return False
    
def sleep(milisecounds):
    time.sleep(milisecounds / 1000)
