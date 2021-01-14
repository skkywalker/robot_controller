import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import cascaded_union

import rospy
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Node:
    def __init__(self, parent, r, theta, is_root=False):
        if parent: parent.children.append(self)
        self.parent = parent
        self.children = []
        self.r = r
        self.theta = theta
        self.is_root = is_root
        self.ctg = 0
        self.is_lowest = False

        self.x = self.r*np.cos(self.theta)
        self.y = self.r*np.sin(self.theta)

class Edge:
    def __init__(self, node_before, node_after):
        self.node_before = node_before
        self.node_after = node_after
        self.x1 = node_before.x
        self.y1 = node_before.y
        self.x2 = node_after.x
        self.y2 = node_after.y
        self.theta = np.arctan2(self.y2-self.y1,self.x2-self.x1)
        self.cost = 0

class Triangle:
    def __init__(self, edge1, edge2):
        node_before = edge1.node_before
        node_after1 = edge1.node_after
        node_after2 = edge2.node_after

        point_before = Point(node_before.r * np.cos(node_before.theta),node_before.r * np.sin(node_before.theta))
        point_after1 = Point(node_after1.r * np.cos(node_after1.theta),node_after1.r * np.sin(node_after1.theta))
        point_after2 = Point(node_after2.r * np.cos(node_after2.theta),node_after2.r * np.sin(node_after2.theta))
        
        self.geometry = Polygon([point_after1, point_after2, point_before])
        self.edge1 = edge1
        self.edge2 = edge2

    def check_collision(self, geometry):
        if geometry.intersects(self.geometry):
            self.edge1.cost = float("inf")
            self.edge2.cost = float("inf")
            return True
        return False

class Lattice:
    def __init__(self, K, Nt, Nb, Nl, r0):
        self.K = K
        self.Nt = Nt
        self.Nb = Nb
        self.Nl = Nl
        self.r0 = r0
        self.layers = []
        self.edges = []
        self.build_lattice()

    def build_lattice(self):
        self.root = Node(None, 0, 0, is_root=True)
        self.layers.append(self.root)
        self.init_first_layer()
        for layer_num in range(self.Nl - 1):
            self.create_layer(layer_num + 2)
        self.build_triangles()
            
    def init_first_layer(self):
        nodes = []
        edges = []
        for i in range(self.Nt):
            tmp = Node(self.root,self.r0,(2*np.pi*(i)/self.Nt))
            nodes.append(tmp)
            edges.append(Edge(self.root, tmp))
        self.layers.append(nodes)
        self.edges.append(edges)

    def create_layer(self, layer_index):
        nodes = []
        edges = []
        for i, node in enumerate(self.layers[layer_index-1]):
            for b in range(self.Nb):
                r = self.K**(layer_index-1) #DIFF
                theta = node.theta + (2*np.pi/self.Nt)*(b+1-(self.Nb+1)/2)/((self.Nb-1)**(layer_index-1))
                tmp = Node(node,r,theta)
                nodes.append(tmp)
                edges.append(Edge(node, tmp))
        self.layers.append(nodes)
        self.edges.append(edges)

    def build_triangles(self):
        self.triangles = []
        for edges in self.edges:
            for i,edge in enumerate(edges):
                if i+1 < len(edges):
                    self.triangles.append(Triangle(edge, edges[i+1]))
                else:
                    self.triangles.append(Triangle(edge, edges[0]))

    def find_intercepted_triangles(self, lidar_angle):
        max_radius = self.K**(self.Nl-1)
        triangles = []
        p1 = (0.1*np.cos(lidar_angle),0.1*np.sin(lidar_angle))
        p2 = (max_radius*np.cos(lidar_angle),max_radius*np.sin(lidar_angle))
        line = LineString([p1, p2])
        for tri in self.triangles:
            if line.intersects(tri.geometry):
                triangles.append(tri)
        return triangles


    def calculate_intercept(self, geometry, plot_geometry=False):
        if plot_geometry:
            plt.plot(*geometry.exterior.xy)
        for triangle in self.triangles:
            triangle.check_collision(geometry)

    def apply_node_cost(self, global_goal):
        for layer in self.edges:
            for edge in layer:
                if edge.cost != float("inf"):
                    edge.cost = 1/(np.pi)*abs((global_goal-edge.theta+np.pi)%(2*np.pi)-np.pi)
                edge.node_after.ctg = edge.node_before.ctg + edge.cost
        return self.find_cheapest_path()

    def find_cheapest_path(self):
        lowest = float("inf")
        lowest_node = self.layers[0]
        for node in self.layers[-1]:
            if node.ctg < lowest:
                lowest = node.ctg
                node.is_lowest = True
                if lowest_node:
                    lowest_node.is_lowest = False
                lowest_node = node
        node = lowest_node
        while not node.is_root:
            node.is_lowest = True
            node = node.parent
        for node in self.layers[1]:
            if node.is_lowest: return node.x, node.y
        return 0,0

    def plot(self):
        plt.plot(self.layers[0].x,self.layers[0].y,'k.', zorder=3)
        for layer in self.layers[1:]:
            for node in layer:
                plt.plot(node.x, node.y, 'k.', zorder=3)

        for edges in self.edges:
            for edge in edges:
                plt.plot([edge.x1,edge.x2], \
                    [edge.y1,edge.y2])
        plt.show()

    def plot_node_cost(self):
        plt.plot(self.layers[0].x,self.layers[0].y,'k.', zorder=3)
        i = 1
        for layer in self.layers[1:]:
            for node in layer:
                if node.is_lowest and i == 1:
                    plt.plot(node.x, node.y, color=(0, 0, 1), marker='.', zorder=3)
                elif node.ctg != float("inf"):
                    plt.plot(node.x, node.y, color=(node.ctg/self.Nl, 0.5, 0), marker='.', zorder=3)
                else:
                    plt.plot(node.x, node.y, color=(1, 0, 0), marker='.', zorder=3)
            i += 1

        for edges in self.edges:
            for edge in edges:
                if edge.cost == float("inf"):
                    plt.plot([edge.x1,edge.x2], [edge.y1,edge.y2], color=(1,0,0))
                else:
                    plt.plot([edge.x1,edge.x2], [edge.y1,edge.y2], color=(edge.cost,0,0))
        plt.show()

    def plot_triangles(self):
        for triangle in self.triangles:
            plt.plot(*triangle.geometry.exterior.xy)
        plt.show()

    def reset_edges_cost(self):
        for edges in self.edges:
            for edge in edges:
                edge.cost = 0

class Controller:
    def __init__(self, lidar_step, robot_radius, robot_offset=[0,0]):
        self.lattice = Lattice(2, 16, 3, 3, 1)
        self.lidar_step = lidar_step # in radians
        self.robot_radius = robot_radius
        self.robot_offset = robot_offset
        self.global_goal = 0

    def apply_lidar(self, lidar_measurements):
        geometry = self.create_geometry(lidar_measurements, 4)
        self.lattice.calculate_intercept(geometry, plot_geometry=False)
        return self.lattice.apply_node_cost(self.global_goal)

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

controller = Controller(0.017501922324299812, 0.15)
x = 0
y = 0

def laser_callback(data : LaserScan):
    global controller, x, y
    m = list(data.ranges)
    for i, _ in enumerate(m):
        if(_ == float("inf")):
            m[i] = 10
    x, y = controller.apply_lidar(m)
    print(x,y)

print("Initializing controller...")


rospy.init_node('controller', anonymous=True)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.Subscriber("/scan", LaserScan, laser_callback)
get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

get_model_state('turtlebot3_burger','')


rate = rospy.Rate(10)

while not rospy.is_shutdown():
    
    rate.sleep()
