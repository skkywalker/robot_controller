import numpy as np
from lattice import Lattice

class Controller:
    def __init__(self, lidar_step, robot_radius, robot_offset=[0,0]):
        self.lattice = Lattice(2, 16, 3, 3, 1)
        self.lidar_step = lidar_step # in radians
        self.robot_radius = robot_radius
        self.robot_offset = robot_offset

    def apply_lidar(self, lidar_measurements):
        for i, measure in enumerate(lidar_measurements):
            if measure > self.lattice.K ** (self.lattice.Nl-1):
                continue
            center_x = measure*np.cos(i*self.lidar_step)
            center_y = measure*np.sin(i*self.lidar_step)
            self.lattice.calculate_intercept([center_x, center_y], self.robot_radius, i*self.lidar_step)
        self.lattice.apply_node_cost()