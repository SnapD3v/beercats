# from realtime_oop import go_back, GO_FORWARD, CORRECT_TIME, TURN_LEFT, TURN_RIGHT, COORDINATE_DIFFERENCE_FOR_CORRECT, time_to_1step_forawrd, true_vector
from config import *


def correct_direction(vector, robot_center, prev_robot_center, connection):
    if prev_robot_center:
        prev_robot_center_x = prev_robot_center[0]
        prev_robot_center_y = prev_robot_center[1]
    else:
        prev_robot_center_x = robot_center[0]
        prev_robot_center_y = robot_center[1]

    robot_center_x = robot_center[0]
    robot_center_y = robot_center[1]

    sleep(100)
    if vector == 0:
        if robot_center_y - prev_robot_center_y > COORDINATE_DIFFERENCE_FOR_CORRECT:
            do(connection, CORRECT_TIME, TURN_LEFT)
            print("Корректировка влево")
        elif robot_center_y - prev_robot_center_y < -COORDINATE_DIFFERENCE_FOR_CORRECT:
            do(connection, CORRECT_TIME, TURN_RIGHT)
            print("Корректировка вправо")
    elif vector == 1:
        if robot_center_x - prev_robot_center_x > COORDINATE_DIFFERENCE_FOR_CORRECT*2:
            do(connection, CORRECT_TIME, TURN_RIGHT)
            print("Корректировка вправо")
        elif robot_center_x - prev_robot_center_x < -COORDINATE_DIFFERENCE_FOR_CORRECT*2:
            do(connection, CORRECT_TIME, TURN_LEFT)
            print("Корректировка влево")
    elif vector == -1:
        if robot_center_x - prev_robot_center_x > COORDINATE_DIFFERENCE_FOR_CORRECT*2:
            do(connection, CORRECT_TIME, TURN_LEFT)
            print("Корректировка влево")
        elif robot_center_x - prev_robot_center_x < -COORDINATE_DIFFERENCE_FOR_CORRECT*2:
            do(connection, CORRECT_TIME, TURN_RIGHT)
            print("Корректировка вправо")
    elif vector == -2:
        if robot_center_y - prev_robot_center_y > COORDINATE_DIFFERENCE_FOR_CORRECT:
            do(connection, CORRECT_TIME, TURN_RIGHT)
            print("Корректировка вправо")
        elif robot_center_y - prev_robot_center_y < -COORDINATE_DIFFERENCE_FOR_CORRECT:
            do(connection, CORRECT_TIME, TURN_LEFT)
            print("Корректировка влево")


def find_true_vector(prev_robot_center, robot_center):
    true_vector = 0
    if prev_robot_center:
        prev_robot_center_x = prev_robot_center[0]
        prev_robot_center_y = prev_robot_center[1]
    robot_center_x = robot_center[0]
    robot_center_y = robot_center[1]
    # Проверка деления на ноль
    if abs(robot_center_y - prev_robot_center_y) != 0:
        # Двигается влево / вправо
        if abs(robot_center_x - prev_robot_center_x) / abs(robot_center_y - prev_robot_center_y) > 3:
            print("1) Движение по оси X")
            # Направлен вправо
            if robot_center_x - prev_robot_center_x > 0:
                true_vector = 0
            # Направлен влево
            elif robot_center_x - prev_robot_center_x < 0:
                true_vector = -2

    # Проверка деления на ноль
    if abs(robot_center_x - prev_robot_center_x) != 0:
        # Двигается вверх / вниз
        if abs(robot_center_y - prev_robot_center_y) / abs(robot_center_x - prev_robot_center_x) > 3:
            print("1) Движение по оси Y")
            # Направлен вниз
            if robot_center_y - prev_robot_center_y > 0:  # новый центр > предыдущего => новый центр ниже
                true_vector = 1  # вниз
            # Направлен вверх
            elif robot_center_y - prev_robot_center_y < 0:  # новый центр < предыдущего => новый центр выше
                true_vector = -1
    return true_vector
