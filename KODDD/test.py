import time
import socket
from config import HOST, PORT, LEFT_SPEED, RIGHT_SPEED, TURN_LEFT, TURN_RIGHT, STOP_COMMAND


class RobotCalibrator:
    def __init__(self):
        self.host = HOST
        self.port = PORT
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
            time.sleep(0.1)  # Small delay between commands
        except socket.error as e:
            print(f"Socket error: {e}")
            self.close()

    def set_speed(self, left_speed, right_speed):
        self.send_command(b'\xff\x02\x01' + bytes([left_speed]) + b'\xff')
        self.send_command(b'\xff\x02\x02' + bytes([right_speed]) + b'\xff')

    def test_turn(self, turn_time, turn_command, turn_name):
        print(f"Testing {turn_name} turn for {turn_time}ms")
        self.send_command(turn_command)
        time.sleep(turn_time / 1000)
        self.send_command(STOP_COMMAND)
        input("Press Enter after measuring the actual turn angle...")

    def run_calibration(self):
        print("=== Robot Turn Timing Calibration ===")
        print("1. Test 90 degree turn")
        print("2. Test 180 degree turn")
        print("3. Custom turn time test")
        print("4. Exit")

        while True:
            choice = input("Select option (1-4): ")

            if choice == "1":
                turn_time = int(input("Enter test time for 90° turn (ms): "))
                self.test_turn(turn_time, TURN_LEFT, "LEFT 90°")
                self.test_turn(turn_time, TURN_RIGHT, "RIGHT 90°")
            elif choice == "2":
                turn_time = int(input("Enter test time for 180° turn (ms): "))
                self.test_turn(turn_time, TURN_LEFT, "LEFT 180°")
                time.sleep(1)
                self.test_turn(turn_time, TURN_LEFT, "LEFT 180° (second part)")
                self.test_turn(turn_time, TURN_RIGHT, "RIGHT 180°")
                time.sleep(1)
                self.test_turn(turn_time, TURN_RIGHT,
                               "RIGHT 180° (second part)")
            elif choice == "3":
                turn_time = int(input("Enter custom turn time (ms): "))
                turn_dir = input("Turn direction (L/R): ").upper()
                if turn_dir == "L":
                    self.test_turn(turn_time, TURN_LEFT, f"LEFT {turn_time}ms")
                elif turn_dir == "R":
                    self.test_turn(turn_time, TURN_RIGHT,
                                   f"RIGHT {turn_time}ms")
            elif choice == "4":
                break
            else:
                print("Invalid choice")

        self.close()


if __name__ == "__main__":
    calibrator = RobotCalibrator()
    calibrator.run_calibration()
