from base_controllers.tracked_robot.simulator.tracked_vehicle_simulator import Ground
import numpy as np


class GroundMap:
    def __init__(self, width=4, height=4) -> None:
        self.width = width
        self.height = height

        np.random.seed(13)

        # Initialize the map with Ground objects
        self.map = [[Ground(friction_coefficient=0.05+np.random.random()*0.15)
                     for _ in range(width)] for _ in range(height)]
        # self.map = [[Ground(friction_coefficient=0.08)
        #              for _ in range(width)] for _ in range(height)]

        # Calculate the coordinate range
        self.x_min = -width // 2
        self.x_max = width // 2
        self.y_min = -height // 2
        self.y_max = height // 2

        self.i_max = height - 1
        self.j_max = width - 1

    def coords_to_index(self, x, y):

        # Convert coordinates to indices
        grid_x = int(np.clip(x - self.x_min, 0, self.width-1))
        grid_y = int(np.clip(y - self.y_min, 0, self.height-1))
        # grid_y = int(np.clip(self.y_max - y, 0, self.height-1))

        return grid_y, grid_x

    def get_ground(self, x, y):

        grid_y, grid_x = self.coords_to_index(x, y)
        return self.map[grid_y][grid_x]
