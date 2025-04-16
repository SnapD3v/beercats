import socket
import time
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("ROBOT")

host = "192.168.1.1"  # Замените на IP робота
port = 2001

def send_command(command, delay=1):
    try:
        logger.debug(f"⌛ Trying to connect to {host}:{port}")

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(3)
        s.connect((host, port))

        logger.info(f"🔧 Sending command: {command.hex()}")
        s.sendall(command)

        if delay > 0:
            logger.debug(f"⏳ Waiting {delay} sec...")
            time.sleep(delay)

        return True
    except socket.timeout:
        logger.error("⌛ Connection timeout!")
        return False
    except socket.error as e:
        logger.error(f"🔌 Socket error: {str(e)}")
        return False
    finally:
        try:
            s.close()
            logger.debug("🔌 Connection closed")
        except:
            pass

def set_speed(left_speed, right_speed):
    send_command(b'\xff\x02\x01' + bytes([left_speed]) + b'\xff', delay=0.5)
    send_command(b'\xff\x02\x02' + bytes([right_speed]) + b'\xff', delay=0.5)

def move_forward(duration=1):
    send_command(b'\xff\x00\x01\x00\xff', delay=duration)
    send_command(b'\xff\x00\x00\x00\xff', delay=0.5)  # Остановка

def turn_left(duration=1):
    send_command(b'\xff\x00\x04\x00\xff', delay=duration)
    send_command(b'\xff\x00\x00\x00\xff', delay=0.5)

def turn_right(duration=1):
    send_command(b'\xff\x00\x03\x00\xff', delay=duration)
    send_command(b'\xff\x00\x00\x00\xff', delay=0.5)