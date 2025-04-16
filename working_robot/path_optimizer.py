import math
import numpy as np


def ramer_douglas_peucker(points, epsilon):
    """Упрощение пути алгоритмом Рамера-Дугласа-Пекера"""
    if len(points) < 3:
        return points

    dmax = 0
    index = 0
    end = len(points) - 1

    for i in range(1, end):
        d = perpendicular_distance(points[i], points[0], points[end])
        if d > dmax:
            index = i
            dmax = d

    if dmax > epsilon:
        left = ramer_douglas_peucker(points[:index + 1], epsilon)
        right = ramer_douglas_peucker(points[index:], epsilon)
        return left[:-1] + right

    return [points[0], points[end]]


def perpendicular_distance(point, line_start, line_end):
    """Расстояние от точки до прямой"""
    x, y = point
    x1, y1 = line_start
    x2, y2 = line_end

    numerator = abs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1)
    denominator = math.hypot(y2 - y1, x2 - x1)
    return numerator / denominator if denominator != 0 else 0