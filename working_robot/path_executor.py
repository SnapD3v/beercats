import math
import logging
from robot_control import move_forward, turn_left, turn_right

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("PATH")


class PathExecutor:
    def __init__(self, path):
        self.path = path
        self.current_angle = 0
        logger.info(f"🛣️ Initialized with path: {path}")

    def _calculate_angle(self, current, next_point):
        dx = next_point[0] - current[0]
        dy = next_point[1] - current[1]
        target_angle = math.degrees(math.atan2(dy, dx))
        angle_diff = target_angle - self.current_angle
        logger.debug(f"")
        logger.debug(f"🧭 Angle diff: {angle_diff:.1f}°")
        return angle_diff

    def execute_path(self):
        logger.info("🏁 Starting path execution")
        try:
            for i in range(len(self.path) - 1):
                current = self.path[i]
                next_point = self.path[i + 1]
                logger.debug(f"📍 Step {i + 1}: {current} → {next_point}")

                angle_diff = self._calculate_angle(current, next_point)

                # Поворот
                if abs(angle_diff) > 5:  # Порог 5 градусов
                    print(angle_diff)
                    print(angle_diff * 0.01)
                    if angle_diff > 0:
                        logger.info(f"🔄 Turning RIGHT by {angle_diff:.1f}°")
                        turn_right(duration=angle_diff * 0.01)
                    else:
                        logger.info(f"🔄 Turning LEFT by {abs(angle_diff):.1f}°")
                        turn_left(duration=abs(angle_diff) * 0.01)
                    self.current_angle += angle_diff

                # Движение
                distance = math.hypot(next_point[0] - current[0], next_point[1] - current[1])
                logger.info(f"🚗 Moving forward {distance:.2f} units")
                move_forward(duration=distance * 0.17)  # Минимум 0.5 сек

        except Exception as e:
            logger.error(f"💥 Execution failed: {str(e)}")