import numpy as np
from lattice import Lattice
from shapely.geometry import Point
from shapely.ops import cascaded_union
import matplotlib.pyplot as plt

class Controller:
    def __init__(self, lidar_step, robot_radius, robot_offset=[0,0]):
        self.lattice = Lattice(2, 16, 3, 3, 1)
        self.lidar_step = lidar_step # in radians
        self.robot_radius = robot_radius
        self.robot_offset = robot_offset

    def apply_lidar(self, lidar_measurements):
        geometry = self.create_geometry(lidar_measurements, 4)
        self.lattice.calculate_intercept(geometry)
        self.lattice.apply_node_cost()

    def create_geometry(self, values, max_val):
        theta = 0
        circles_to_join = []
        for i in values:
            if i > max_val:
                theta += self.lidar_step
                continue
            x = i*np.cos(theta)
            y = i*np.sin(theta)
            tmp_pol = Point(x,y).buffer(self.robot_radius)
            circles_to_join.append(tmp_pol)
            theta += self.lidar_step
        return cascaded_union(circles_to_join)