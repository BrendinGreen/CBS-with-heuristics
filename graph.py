class Vertex:
    def __init__(self, vertex):
        self.name = vertex
        self.neighbors = []

    def add_neighbor(self, neighbor):
        if isinstance(neighbor, Vertex):  # check if neighbor is vertex type
            if neighbor.name not in self.neighbors:
                self.neighbors.append(neighbor.name)
                neighbor.neighbors.append(self.name)
                self.neighbors = sorted(self.neighbors)
                neighbor.neighbors = sorted(neighbor.neighbors)
        else:
            return False

    def add_neighbors(self, neighbors):
        for neighbor in neighbors:
            if isinstance(neighbor, Vertex):  # check if neighbor is vertex type
                if neighbor.name not in self.neighbors:
                    self.neighbors.append(neighbor.name)
                    neighbor.neighbors.append(self.name)
                    self.neighbors = sorted(self.neighbors)
                    neighbor.neighbors = sorted(neighbor.neighbors)
            else:
                return False

    def __repr__(self):
        return str(self.neighbors)


class Graph:
    def __init__(self):
        self.vertices = {}
        self.edges = set()

    def add_vertex(self, vertex):
        if isinstance(vertex, Vertex):
            self.vertices[vertex.name] = vertex.neighbors

    def add_vertices(self, vertices):
        for vertex in vertices:
            if isinstance(vertex, Vertex):
                self.vertices[vertex.name] = vertex.neighbors

    def add_edge(self, vertex_from, vertex_to):
        if isinstance(vertex_from, Vertex) and isinstance(vertex_to, Vertex):
            vertex_from.add_neighbor(vertex_to)
            if isinstance(vertex_from, Vertex) and isinstance(vertex_to, Vertex):
                self.vertices[vertex_from.name] = vertex_from.neighbors
                self.vertices[vertex_to.name] = vertex_to.neighbors
                self.edges.add(
                    tuple(sorted([vertex_from.name, vertex_to.name])))

    def add_edges(self, edges):
        for edge in edges:
            self.add_edge(edge[0], edge[1])

    def number_of_vertrices(self):
        return len(self.vertices)

    def number_of_edges(self):
        return len(self.edges)

    def adjacencyList(self):
        if len(self.vertices) >= 1:
            return [str(key) + ":" + str(self.vertices[key]) for key in self.vertices.keys()]
        else:
            return dict()

    def remove_vertex(self, vertex):
        # remove neighbours
        rm_list = []
        for edge in self.edges:
            if vertex.name == edge[0]:
                rm_list.append(edge[1])
            elif vertex.name == edge[1]:
                rm_list.append(edge[0])
        # remove edge
        new_edge = {
            edge for edge in self.edges if vertex.name not in (edge[0], edge[1])}
        self.edges = new_edge

        # remove vertex
        if vertex.name in self.vertices:
            del self.vertices[vertex.name]
        for v in rm_list:
            self.vertices[v].remove(vertex.name)

    def adjacencyMatrix(self):
        if len(self.vertices) >= 1:
            self.vertex_names = sorted(g.vertices.keys())
            self.vertex_indices = dict(
                zip(self.vertex_names, range(len(self.vertex_names))))
            import numpy as np
            self.adjacency_matrix = np.zeros(
                shape=(len(self.vertices), len(self.vertices)))
            for i in range(len(self.vertex_names)):
                for j in range(i, len(self.vertices)):
                    for el in g.vertices[self.vertex_names[i]]:
                        j = g.vertex_indices[el]
                        self.adjacency_matrix[i, j] = 1
            return self.adjacency_matrix
        else:
            return dict()


def graph(g):
    """ Function to print a graph as adjacency list and adjacency matrix. """
    return str(g.adjacencyList()) + '\n' + '\n' + str(g.adjacencyMatrix())

###################################################################################


a = Vertex('1')
b = Vertex('2')
c = Vertex('5')
d = Vertex('6')
e = Vertex('7')

a.add_neighbors([2])
b.add_neighbors([1])
c.add_neighbors([6, 7])
d.add_neighbors([5, 7])
e.add_neighbors([5, 6])


g = Graph()
print(graph(g))
print("\n")
g.add_vertices([a, b, c, d, e])
g.add_edges([(a, b), (e, d), (d, c), (c, e)])
print("#edge:", g.number_of_edges())
print("#vertex:", g.number_of_vertrices())
print("remove vertex 5")
g.remove_vertex(c)
print("#edge:", g.number_of_edges())
print("#vertex:", g.number_of_vertrices())
print("\n")
print(graph(g))
