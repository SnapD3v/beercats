import socket
import time

class Robot:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.connection = None
        self.commands = {
            'forward': b"\xff\x00\x01\x00\xff",
            'backward': b"\xff\x00\x02\x00\xff",
            'right': b"\xff\x00\x04\x00\xff",
            'left': b"\xff\x00\x03\x00\xff",
            'stop': b"\xff\x00\x00\x00\xff"
        }
        
    def connect(self):
        try:
            self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.connection.connect((self.host, self.port))
            print("Connected to robot")
        except socket.error as e:
            print(f"Connection error: {e}")

    def send_command(self, command, duration_ms):
        try:
            self.connection.sendall(command)
            self.sleep(duration_ms)
            self.connection.sendall(self.commands['stop'])
        except Exception as e:
            print(f"Command failed: {e}")

    def set_speed(self, left, right):
        self.connection.sendall(b'\xff\x02\x01' + bytes([left]) + b'\xff')
        self.connection.sendall(b'\xff\x02\x02' + bytes([right]) + b'\xff')

    def sleep(self, ms):
        time.sleep(ms / 1000)

    def disconnect(self):
        if self.connection:
            self.connection.close()
            print("Disconnected from robot")