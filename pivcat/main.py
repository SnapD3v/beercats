from navigation import NavigationController
from robot import Robot
from vision_system import VisionSystem
import asyncio
import cv2


class RoboNavigator:
    def __init__(self, config):
        self.config = config
        self.robot = Robot(config['host'], config['port'])
        self.robot.connect()
        self.vision = VisionSystem(config['model_path'])
        self.navigator = NavigationController(self.robot, self.vision, config)

    def run(self):
        cap = cv2.VideoCapture(self.config['rtsp_url'])

        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            frame = self.vision.remove_distortion(frame)
            asyncio.run(self.navigator.process_frame(frame))

        cap.release()
        self.robot.disconnect()


if __name__ == "__main__":
    config = {
        'host': '192.168.1.1',
        'port': 2001,
        'model_path': 'best.pt',
        'rtsp_url': 'rtsp://admin:UrFU_ISIT@10.32.9.223:554/Streaming/channels/101'
    }

    app = RoboNavigator(config)
    app.run()
