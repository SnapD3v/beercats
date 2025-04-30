import cv2
import a_star
import matplotlib.pyplot as plt
from robot_control import set_speed
from path_executor import PathExecutor
from path_optimizer import ramer_douglas_peucker
import frame_processor
from working_robot.frame_processor import detect_objects, get_robot_center
from camera import get_cam_image

# ========== НАСТРОЙКИ ==========
IMAGE_PATH = 'maze1.jpg'  # Путь к готовому изображению
RESIZED_SIZE = (100, 40)  # Размер для обработки
THRESHOLD_VALUE = 120  # Порог бинаризации (0-255)
START = (10, 20)  # Стартовая позиция (x,y)
GOAL = (80, 20)  # Целевая позиция
ROBOT_RADIUS = 2.8  # Радиус робота
GRID_SIZE = 2.0  # Размер ячейки сетки


# ===============================

def process_image(image_path):
    # Загрузка и предобработка изображения
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY)
    resized = cv2.resize(binary, RESIZED_SIZE)
    resized = cv2.flip(resized, 0)
    return resized


def get_obstacles(binary_image):
    # Получение координат препятствий
    x_obs, y_obs = [], []
    height, width = binary_image.shape
    for y in range(height):
        for x in range(width):
            if binary_image[y, x] <= 150:
                x_obs.append(x)
                y_obs.append(y)
    return x_obs, y_obs


# Основной поток выполнения
if __name__ == "__main__":

    img = get_cam_image("rtsp://admin:UrFU_ISIT@10.32.21.105:554/Streaming/channels/101", (100, 40))

    # 1. Обработка изображения
    binary_img = process_image(img)

    # 2. Построение карты препятствий
    x_obstacle, y_obstacle = get_obstacles(binary_img)

    boxes, indexes = detect_objects(img)
    current_pos = get_robot_center(boxes, indexes)
    current_angle = 0

    while current_pos != GOAL:
        img = get_cam_image("rtsp://admin:UrFU_ISIT@10.32.21.105:554/Streaming/channels/101", (100, 40))
        # 1. Обработка изображения
        binary_img = process_image(img)

        # 2. Построение карты препятствий
        x_obstacle, y_obstacle = get_obstacles(binary_img)

        #3. Визуализация
        plt.figure(figsize=(10, 5))
        plt.plot(x_obstacle, y_obstacle, ".k", markersize=2)
        plt.plot(START[0], START[1], "og", label="Start")
        plt.plot(GOAL[0], GOAL[1], "xr", label="Goal")
        plt.legend()
        plt.grid(True)

        boxes, indexes = detect_objects(IMAGE_PATH)
        current_pos = get_robot_center(boxes, indexes)

        # 4. Поиск пути
        planner = a_star.AStarPath(
            robot_radius=ROBOT_RADIUS,
            grid_size=GRID_SIZE,
            x_obstacle=x_obstacle,
            y_obstacle=y_obstacle
        )

        path_x, path_y = planner.a_star_search(START[0], START[1], GOAL[0], GOAL[1])
        path = ramer_douglas_peucker(list(zip(path_x, path_y)), 1.5)
        path = path[1:]
        print(path)
        # 5. Отображение пути
        # plt.plot(path_x, path_y, "-b", linewidth=2)
        # plt.title("A* Path Planning")
        # plt.show()
        opt_x = [p[0] for p in path]
        opt_y = [p[1] for p in path]
        plt.plot(opt_x, opt_y, '-g', linewidth=3, label="Optimized path")
        plt.title("A* Path Planning")
        plt.show()

        # 6. Исполнение пути на роботе
        set_speed(0x2D, 0x29)
        print("ITERATION:" + " POS: " + str(current_pos[0]) + " " + str(current_pos[0]) + " ANGLE: " + str(current_angle))
        executor = PathExecutor(path, current_angle)
        executor.execute_path()
        current_angle = executor.current_angle