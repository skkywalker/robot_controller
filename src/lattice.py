import numpy as np

class Node:
    def __init__(self, parent, r, theta, is_root=False):
        self.parent = parent
        self.r = r
        self.theta = theta
        self.is_root = is_root
    
    def __repr__(self):
        if self.is_root:
            return "Root node"
        else:
            return "Radius " + str(self.r) + "; Theta " + str(self.theta)

class Edge:
    def __init__(self, node_before, node_after):
        self.node_before = node_before
        self.node_after = node_after

    def __repr__(self):
        return "Edge from " + str(self.node_before) + " to " + str(self.node_after)

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
        print("Creating layer...")
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