from config import do, sleep, go_back, go_forward, CORRECT_TIME, turn_left, turn_right, COORDINATE_DIFFERENCE_FOR_CORRECT, time_to_1step_forawrd, true_vector


class MovementCorrector:
    def __init__(self, connection):
        self.connection = connection

    def correct_direction(self, vector, robot_center, prev_robot_center):
        if prev_robot_center:
            prev_robot_center_x = prev_robot_center[0]
            prev_robot_center_y = prev_robot_center[1]
        robot_center_x = robot_center[0]
        robot_center_y = robot_center[1]

        sleep(100)
        if vector == 0:
            if robot_center_y - prev_robot_center_y > COORDINATE_DIFFERENCE_FOR_CORRECT:
                self._do_correction(turn_left, "Корректировка влево")
            elif robot_center_y - prev_robot_center_y < -COORDINATE_DIFFERENCE_FOR_CORRECT:
                self._do_correction(turn_right, "Корректировка вправо")
        elif vector == 1:
            if robot_center_x - prev_robot_center_x > COORDINATE_DIFFERENCE_FOR_CORRECT * 2:
                self._do_correction(turn_right, "Корректировка вправо")
            elif robot_center_x - prev_robot_center_x < -COORDINATE_DIFFERENCE_FOR_CORRECT * 2:
                self._do_correction(turn_left, "Корректировка влево")
        elif vector == -1:
            if robot_center_x - prev_robot_center_x > COORDINATE_DIFFERENCE_FOR_CORRECT * 2:
                self._do_correction(turn_left, "Корректировка влево")
            elif robot_center_x - prev_robot_center_x < -COORDINATE_DIFFERENCE_FOR_CORRECT * 2:
                self._do_correction(turn_right, "Корректировка вправо")
        elif vector == -2:
            if robot_center_y - prev_robot_center_y > COORDINATE_DIFFERENCE_FOR_CORRECT:
                self._do_correction(turn_right, "Корректировка вправо")
            elif robot_center_y - prev_robot_center_y < -COORDINATE_DIFFERENCE_FOR_CORRECT:
                self._do_correction(turn_left, "Корректировка влево")

    def _do_correction(self, direction, message):
        do(self.connection, CORRECT_TIME, direction)
        print(message)

    def find_true_vector(self, prev_robot_center, robot_center):
        true_vector = 0
        if prev_robot_center:
            prev_robot_center_x = prev_robot_center[0]
            prev_robot_center_y = prev_robot_center[1]
        robot_center_x = robot_center[0]
        robot_center_y = robot_center[1]

        # Check for division by zero
        if abs(robot_center_y - prev_robot_center_y) != 0:
            # Moving left / right
            if abs(robot_center_x - prev_robot_center_x) / abs(robot_center_y - prev_robot_center_y) > 3:
                print("1) Движение по оси X")
                # Facing right
                if robot_center_x - prev_robot_center_x > 0:
                    true_vector = 0
                # Facing left
                elif robot_center_x - prev_robot_center_x < 0:
                    true_vector = -2

        # Check for division by zero
        if abs(robot_center_x - prev_robot_center_x) != 0:
            # Moving up / down
            if abs(robot_center_y - prev_robot_center_y) / abs(robot_center_x - prev_robot_center_x) > 3:
                print("1) Движение по оси Y")
                # Facing down
                if robot_center_y - prev_robot_center_y > 0:
                    true_vector = 1
                # Facing up
                elif robot_center_y - prev_robot_center_y < 0:
                    true_vector = -1

        return true_vector
