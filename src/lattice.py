import numpy as np
from shapely.geometry import Point, Polygon, LineString

class Node:
    def __init__(self, parent, r, theta, is_root=False):
        self.parent = parent
        self.r = r
        self.theta = theta
        self.is_root = is_root
        self.cost_to_go = 0
    
    def __repr__(self):
        if self.is_root:
            return "Root node"
        else:
            return "Radius " + str(self.r) + "; Theta " + str(self.theta)

class Edge:
    def __init__(self, node_before, node_after):
        self.node_before = node_before
        self.node_after = node_after
        self.cost = 0

    def __repr__(self):
        return "Edge from " + str(self.node_before) + " to " + str(self.node_after)

class Triangle:
    def __init__(self, edge1, edge2):
        node_before = edge1.node_before
        node_after1 = edge1.node_after
        node_after2 = edge2.node_after

        self.edge1 = edge1
        self.edge2 = edge2

        point_before = Point(node_before.r * np.cos(node_before.theta),node_before.r * np.sin(node_before.theta))
        point_after1 = Point(node_after1.r * np.cos(node_after1.theta),node_after1.r * np.sin(node_after1.theta))
        point_after2 = Point(node_after2.r * np.cos(node_after2.theta),node_after2.r * np.sin(node_after2.theta))
        self.geometry = Polygon([point_after1, point_after2, point_before])
        self.node_before = node_before
        self.node_after1 = node_after1
        self.node_after2 = node_after2

    def collision(self, center, radius):
        circle = Point(center).buffer(radius)
        return circle.intersects(self.geometry)

    def apply_collision(self, center, radius):
        if self.collision(center, radius):
            self.node_after1.cost_to_go = 999
            self.node_after2.cost_to_go = 999
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
        self.init_lattice()

    def init_lattice(self):
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
            tri = []
            for i,edge in enumerate(edges):
                if i+1 < len(edges):
                    tri.append(Triangle(edge, edges[i+1]))
                else:
                    tri.append(Triangle(edge, edges[0]))
            self.triangles.append(tri)

    def find_intercepted_triangles(self, lidar_angle):
        max_radius = self.K**(self.Nl-1)
        triangles = []
        p1 = (0.1*np.cos(lidar_angle),0.1*np.sin(lidar_angle))
        p2 = (max_radius*np.cos(lidar_angle),max_radius*np.sin(lidar_angle))
        line = LineString([p1, p2])
        for tris in self.triangles:
            for triangle in tris:
                if line.intersects(triangle.geometry):
                    triangles.append(triangle)
        return triangles


    def calculate_intercept(self, center, radius, lidar_angle):
        triangles = self.find_intercepted_triangles(lidar_angle)
        for triangle in triangles:
            triangle.apply_collision(center, radius)

    def plot(self):
        import matplotlib.pyplot as plt
        plt.polar(self.layers[0].theta,self.layers[0].r,'k.', zorder=3)
        for layer in self.layers[1:]:
            for node in layer:
                plt.polar(node.theta, node.r, 'k.', zorder=3)

        for edges in self.edges:
            for edge in edges:
                plt.polar([edge.node_before.theta,edge.node_after.theta], \
                    [edge.node_before.r,edge.node_after.r])
        plt.show()

    def plot_triangles(self):
        import matplotlib.pyplot as plt

        for triangles in self.triangles:
            for triangle in triangles:
                plt.plot(*triangle.geometry.exterior.xy)
        plt.show()