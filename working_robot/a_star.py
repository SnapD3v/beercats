import math
import matplotlib.pyplot as plt

show_animation = False

class AStarPath:
    def __init__(self, robot_radius, grid_size, x_obstacle, y_obstacle):
        self.grid_size = grid_size
        self.robot_radius = robot_radius
        self.create_obstacle_map(x_obstacle, y_obstacle)
        self.path = self.get_path()

    class Node:
        def __init__(self, x, y, cost, path):
            self.x = x
            self.y = y
            self.cost = cost
            self.path = path
        
        def __str__(self):
            return str(self.x)+'+'+str(self.y)+','+str(self.cost)+','+str(self.path)
    
    @staticmethod
    def calc_heuristic(num1, num2):
        weight = 1.0
        return weight * math.sqrt((num1.x-num2.x)**2 + (num1.y-num2.y)**2)

    def calc_grid_position(self, idx, p):
        return idx * self.grid_size + p

    def calc_xy(self, position, min_position):
        return round((position-min_position) / self.grid_size)
    
    def calc_grid_idx(self, node):
        return (node.y-self.y_min) * self.x_width + (node.x-self.x_min)
    
    def check_validity(self, node):
        x_position = self.calc_grid_position(node.x, self.x_min)
        y_position = self.calc_grid_position(node.y, self.y_min)

        if x_position < self.x_min:
            return False
        elif y_position < self.y_min:
            return False
        elif x_position >= self.x_max:
            return False
        elif y_position >= self.y_max:
            return False
        if self.obstacle_pos[node.x][node.y]:
            return False
        return True

    def create_obstacle_map(self, x_obstacle, y_obstacle):
        self.x_min = round(min(x_obstacle))
        self.y_min = round(min(y_obstacle))
        self.x_max = round(max(x_obstacle))
        self.y_max = round(max(y_obstacle))
        self.x_width = round((self.x_max - self.x_min) / self.grid_size)
        self.y_width = round((self.y_max - self.y_min) / self.grid_size)
        self.obstacle_pos = [[False for i in range(self.y_width)] for i in range(self.x_width)]
        for idx_x in range(self.x_width):
            x = self.calc_grid_position(idx_x, self.x_min)
            for idx_y in range(self.y_width):
                y = self.calc_grid_position(idx_y, self.y_min)
                for idx_x_obstacle, idx_y_obstacle in zip(x_obstacle, y_obstacle):
                    d = math.sqrt((idx_x_obstacle - x)**2 + (idx_y_obstacle - y)**2)
                    if d <= self.robot_radius:
                        self.obstacle_pos[idx_x][idx_y] = True
                        break
    
    @staticmethod
    def get_path():
        path = [[1, 0, 1],
                [0, 1, 1],
                [-1, 0, 1],
                [0, -1, 1],
                [-1, -1, math.sqrt(2)],
                [-1, 1, math.sqrt(2)],
                [1, -1, math.sqrt(2)],
                [1, 1, math.sqrt(2)]]

        return path

    def calc_final_path(self, end_node, record_closed):
        path = []
        current_node = end_node
        while current_node.path != -1:
            path.append((
                self.calc_grid_position(current_node.x, self.x_min),
                self.calc_grid_position(current_node.y, self.y_min)
            ))
            current_node = record_closed[current_node.path]

        # Добавляем стартовую точку
        path.append((
            self.calc_grid_position(current_node.x, self.x_min),
            self.calc_grid_position(current_node.y, self.y_min)
        ))

        # Разворачиваем путь от старта к цели
        path = path[::-1]
        x_out = [p[0] for p in path]
        y_out = [p[1] for p in path]
        return x_out, y_out

        return x_out_path, y_out_path

    def a_star_search(self, start_x, start_y, end_x, end_y):
        start_node = self.Node(self.calc_xy(start_x, self.x_min), self.calc_xy(start_y, self.y_min), 0.0, -1)
        end_node = self.Node(self.calc_xy(end_x, self.x_min), self.calc_xy(end_y, self.y_min), 0.0, -1)

        record_open, record_closed = dict(), dict()
        record_open[self.calc_grid_idx(start_node)] = start_node

        while True:
            if len(record_open) == 0:
                print('Check Record Validity')
                break

            total_cost = min(record_open, key=lambda x: record_open[x].cost + self.calc_heuristic(end_node, record_open[x]))
            cost_collection = record_open[total_cost]
        
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(cost_collection.x, self.x_min),  self.calc_grid_position(cost_collection.y, self.y_min), "xy")
                if len(record_closed.keys())%10 == 0:
                    plt.pause(0.001)
            
            if cost_collection.x == end_node.x and cost_collection.y == end_node.y:
                print("Finished!")
                end_node.path = cost_collection.path
                end_node.cost = cost_collection.cost
                break

            del record_open[total_cost]
            record_closed[total_cost] = cost_collection

            for i, _ in enumerate(self.path):
                node = self.Node(cost_collection.x + self.path[i][0], cost_collection.y + self.path[i][1], cost_collection.cost + self.path[i][2], total_cost)
                idx_node = self.calc_grid_idx(node)

                if not self.check_validity(node):
                    continue

                if idx_node in record_closed:
                    continue

                if idx_node not in record_open:
                    record_open[idx_node] = node
                else:
                    if record_open[idx_node].cost > node.cost:
                        record_open[idx_node] = node

        x_out_path, y_out_path = self.calc_final_path(end_node, record_closed)

        return x_out_path, y_out_path