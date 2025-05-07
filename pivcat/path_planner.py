import cv2
import numpy as np
from astar import AStar

class PathPlanner(AStar):
    def __init__(self, obstacle_map):
        super().__init__()
        self.obstacle_map = obstacle_map

    def heuristic_cost_estimate(self, n1, n2):
        return abs(n1[0] - n2[0]) + abs(n1[1] - n2[1])

    def neighbors(self, node):
        x, y = node
        neighbors = []
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < self.obstacle_map.shape[1] and 0 <= ny < self.obstacle_map.shape[0]:
                if self.obstacle_map[ny, nx] == 0:
                    neighbors.append((nx, ny))
        return neighbors

    def plan_path(self, start, goal):
        try:
            return list(self.astar(start, goal))
        except Exception as e:
            print(f"Path planning error: {e}")
            return None