import cv2
import numpy as np

camera_matrix = np.array([[7.99605236e+02, 0.00000000e+00, 1.26637524e+03],
       [0.00000000e+00, 7.94583638e+02, 9.54402653e+02],
       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist_coefs = np. array([-0.6836352 ,  0.54420523, -0.00392804,  0.00639044, -0.22176408])

def get_cam_image(rtps_url, img_size):
    # Создаем объект для захвата видео
    cap = cv2.VideoCapture(rtps_url)

    # Проверяем успешность подключения
    if not cap.isOpened():
        print("Ошибка: Не удалось подключиться к камере")
        exit()

    try:
        # Считываем кадр из потока
        ret, frame = cap.read()

        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coefs, (w, h), 0, (w, h))

        dst = cv2.undistort(frame, camera_matrix, dist_coefs, None, newcameramtx)


        points = np.array([
            [500, 750],  # Левый верхний
            [500, 1072],  # Левый нижний
            [1929, 750],  # Правый верхний (исправлено: 586 вместо 742)
            [1929, 1060]  # Правый нижний
        ], dtype=np.int32)
        x, y, w, h = cv2.boundingRect(points)
        dst = dst[y:y + h, x:x + w]

        if not ret:
            print("Ошибка: Не удалось получить кадр")

        cv2.imwrite('maze1.png', dst)
        # Отображаем кадр в окне

        # Прерывание по нажатию 'q'
    finally:
        # Освобождаем ресурсы и закрываем окна
        cap.release()
        cv2.destroyAllWindows()
