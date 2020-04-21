import networkx as nx

def mvc(g):
    for k in range(1, g.number_of_nodes()):
        if (k_vertex_cover(g, k)):
            return k


def k_vertex_cover(g, k):

    if (g.number_of_edges() == 0):
        return True
    elif (g.number_of_edges() > k*g.number_of_nodes()):
        return False

    # pick any edge in graph
    v = list(g.edges())[0]
    g1 = g.copy()
    g2 = g.copy()

    g1.remove_node(v[0])
    g2.remove_node(v[1])
    # recursively check if either g1 or g2 have vertex cover of k-1
    return k_vertex_cover(g1, k-1) or k_vertex_cover(g2, k-1)
