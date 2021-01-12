import numpy as np
from shapely.geometry import Point, Polygon, LineString
import matplotlib.pyplot as plt

class Node:
    def __init__(self, parent, r, theta, is_root=False):
        if parent: parent.children.append(self)
        self.parent = parent
        self.children = []
        self.r = r
        self.theta = theta
        self.is_root = is_root
        self.ctg = 0

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

    def check_collision(self, center, radius):
        circle = Point(center).buffer(radius)
        if circle.intersects(self.geometry):
            self.edge1.cost = 999
            self.edge2.cost = 999
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


    def calculate_intercept(self, center, radius, lidar_angle):
        triangles = self.find_intercepted_triangles(lidar_angle)
        for triangle in triangles:
            triangle.check_collision(center, radius)

    def apply_node_cost(self):
        for layer in self.edges:
            for edge in layer:
                edge.node_after.ctg = edge.node_before.ctg + edge.cost

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
        for layer in self.layers[1:]:
            for node in layer:
                if node.ctg > 0:
                    plt.plot(node.x, node.y, 'r.', zorder=3)
                else:
                    plt.plot(node.x, node.y, 'k.', zorder=3)

        for edges in self.edges:
            for edge in edges:
                if edge.cost > 0:
                    plt.plot([edge.x1,edge.x2], \
                    [edge.y1,edge.y2], 'r')
                else:
                    plt.plot([edge.x1,edge.x2], \
                    [edge.y1,edge.y2], 'b')
        plt.show()

    def plot_triangles(self):

        for triangle in self.triangles:
            plt.plot(*triangle.geometry.exterior.xy)
        plt.show()

    def reset_edges_cost(self):
        for edges in self.edges:
            for edge in edges:
                edge.cost = 0