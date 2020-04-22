import networkx as nx


def mvc(g):
    for k in range(1, g.number_of_nodes()):
        if k_vertex_cover(g, k):
            return k


def k_vertex_cover(g, k):

    if g.number_of_edges() == 0:
        return True
    elif g.number_of_edges() > k*g.number_of_nodes():
        return False

    # pick any edge in graph
    v = list(g.edges())[0]
    g1 = g.copy()
    g2 = g.copy()

    g1.remove_node(v[0])
    g2.remove_node(v[1])
    # recursively check if either g1 or g2 have vertex cover of k-1
    return k_vertex_cover(g1, k-1) or k_vertex_cover(g2, k-1)


def construct_conflict_graph(num_agents, mdds):

    conflict_graph = nx.Graph()

    for outer_agent in range(num_agents):
        for inner_agent in range(outer_agent + 1, num_agents):
            # print("Comparing: {}, {}".format(outer_agent, inner_agent))
            a1_mdd = mdds[outer_agent]['mdd']
            a2_mdd = mdds[inner_agent]['mdd']
            # print(a1_mdd)
            # print(a2_mdd)
            min_path_length = min(len(a2_mdd.keys()), len(a1_mdd.keys()))

            for timestep in range(min_path_length):
                if len(a1_mdd[timestep]) == 1 and len(a2_mdd[timestep]) == 1:
                    if a1_mdd[timestep][0] == a2_mdd[timestep][0]:
                        conflict_graph.add_edge(inner_agent, outer_agent)

    return conflict_graph
