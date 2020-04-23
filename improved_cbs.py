import time as timer
import os
import psutil
from single_agent_planner import *
from conflict_graph import *


def get_agent_cost(paths):
    rst = []
    for path in paths:
        rst.append(len(path) - 1)
    return rst


def compute_cg_heuristic(my_map, start, goal, num_agents, node):

    mdds = []

    for agent in range(num_agents):
        mdds.append(build_mdd(my_map, start[agent], goal[agent], agent, node['constraints'], node['agent_cost'][agent]))

    conflict_graph = construct_conflict_graph(num_agents, mdds)

    print(conflict_graph)

    hval = mvc(conflict_graph)

    print("h_val: {}".format(hval))

    if hval is None:
        return 0

    return hval


def detect_collision(path1, path2):

    longest_path_length = max(len(path1), len(path2))

    # check for vertex collisions
    for index in range(0, longest_path_length):
        if get_location(path1, index) == get_location(path2, index):
            return {'loc': [get_location(path1, index)], 'timestep': index}

    # check for edge collisions
    for index in range(0, longest_path_length - 1):
        edge1 = [get_location(path1, index), get_location(path1, index + 1)]
        edge2 = [get_location(path2, index), get_location(path2, index + 1)]
        if edge1 == edge2[::-1]:
            return {'loc': edge1, 'timestep': index + 1}

    return None


def detect_collisions(paths):

    collisions = []

    for index1 in range(0, len(paths)):
        for index2 in range(index1 + 1, len(paths)):
            if index1 != index2:
                first_collision = detect_collision(paths[index1], paths[index2])
                if first_collision is not None:
                    first_collision['a1'] = index1
                    first_collision['a2'] = index2
                    collisions.append(first_collision)

    return collisions


def a_star_mdd(my_map, start_loc, goal_loc, agent, constraints, depth):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
    """
    
    def manhattan_dist(node, goal):
        """ Compute Manhattan distance """
        x1 = node[0]
        y1 = node[1]
        x2 = goal[0]
        y2 = goal[1]

        return abs(x1 - x2) + abs(y1 - y2)
    
    open_list = []
    closed_list = dict()
    graph = dict()

    constraint_table = build_constraint_table(constraints, agent)

    h_value = manhattan_dist(start_loc, goal_loc)
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'timestep': 0, 'parent': None}
    graph[root['timestep']] = [root['loc']]
    
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root

    while len(open_list) > 0:
        curr = pop_node(open_list)

        if curr['loc'] == goal_loc:
            continue

        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            
            if (child_loc[0] >= len(my_map) or child_loc[0] == -1) or (child_loc[1] >= len(my_map[0]) or child_loc[1] == -1):
                continue

            if my_map[child_loc[0]][child_loc[1]]:
                continue
            
            # If child node is constrained, do not generate node and remove its parent from graph
            if is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraint_table):
                try:
                    # If the parent is shared by another node, do not remove
                    sibling = graph[curr['timestep'] + 1]
                    continue
                # Otherwise, remove parent
                except KeyError:
                    try:
                        graph[curr['timestep']].remove(curr['loc'])
                        continue
                    except:
                        continue
            
            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,
                     'h_val': manhattan_dist(child_loc, goal_loc),
                     'parent': curr,
                     'timestep': curr['timestep'] + 1}
            
            if (child['loc'], child['timestep']) in closed_list:
                if child['g_val'] + child['h_val'] <= depth:
                    existing_node = closed_list[(child['loc'], child['timestep'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['loc'], child['timestep'])] = child
                        push_node(open_list, child)
                    if child['timestep'] not in graph:
                        graph[child['timestep']] = [child['loc']]
                    else:
                        if child['loc'] not in graph[child['timestep']]:
                            graph[child['timestep']].append(child['loc'])
            else:
                if child['g_val'] + child['h_val'] <= depth:
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
                    if child['timestep'] not in graph:
                        graph[child['timestep']] = [child['loc']]
                    else:
                        if child['loc'] not in graph[child['timestep']]:
                            graph[child['timestep']].append(child['loc'])

    if len(graph) > 0:
        return graph
    else:
        return None


def build_mdd(my_map, start_loc, goal_loc, agent, constraints, depth):
    graph = a_star_mdd(my_map, start_loc, goal_loc, agent, constraints, depth)
    mdd = {'agent': agent, 'mdd': graph}
    return mdd


def detect_cardinal_conflict(mdd1, mdd2):

    mdd1_graph = mdd1['mdd']
    mdd2_graph = mdd2['mdd']

    conflict = {'a1': mdd1['agent'],
                'a2': mdd2['agent']}

    # print(mdd1_graph)
    # print(mdd2_graph)

    min_path_length = min(len(mdd1_graph.keys()), len(mdd2_graph.keys()))

    # First look for cardinal conflicts
    for timestep in range(min_path_length):
        if len(mdd1_graph[timestep]) == 1 and len(mdd2_graph[timestep]) == 1:
            if mdd1_graph[timestep][0] == mdd2_graph[timestep][0]:
                conflict['loc'] = [mdd1_graph[timestep][0]]
                conflict['timestep'] = timestep
                return conflict

    # Second, look for semi-cardinal conflicts
    for timestep in range(min_path_length):
        if len(mdd1_graph[timestep]) == 1 and len(mdd2_graph[timestep]) > 1:
            for loc in mdd2_graph[timestep]:
                if mdd1_graph[timestep][0] == loc:
                    conflict['loc'] = [loc]
                    conflict['timestep'] = timestep
                    return conflict

        if len(mdd1_graph[timestep]) > 1 and len(mdd2_graph[timestep]) == 1:
            for loc in mdd1_graph[timestep]:
                if mdd2_graph[timestep][0] == loc:
                    conflict['loc'] = [loc]
                    conflict['timestep'] = timestep
                    return conflict

    # no cardinal, or semi-cardinal found
    return None


def standard_splitting(collision):

    first_constraint = {'agent': collision['a1'],
                        'loc': collision['loc'],
                        'timestep': collision['timestep'],
                        'positive': False}

    second_constraint = {'agent': collision['a2'],
                         'loc': collision['loc'][::-1],
                         'timestep': collision['timestep'],
                         'positive': False}

    return [first_constraint, second_constraint]


def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


class ICBSSolver(object):
    """The high-level search of Improved CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node, conflict):
        if not conflict:
            heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        else:
            heapq.heappush(self.open_list, (node['h_val'], node['cost'], len(node['collisions']), self.num_of_generated, node))
        self.num_of_generated += 1

    def pop_node(self, conflict):
        if not conflict:
            _, _, id, node = heapq.heappop(self.open_list)
        else:
            _, _, _, id, node = heapq.heappop(self.open_list)
        self.num_of_expanded += 1
        return node

    def find_solution(self, conflict_graph=False):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        print("Conflict: {}".format(conflict_graph))

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths

        # Initialize root with low-level paths for individual agents
        root = {'cost': 0,
                'agent_cost': [],
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map,
                          self.starts[i],
                          self.goals[i],
                          self.heuristics[i],
                          i,
                          root['constraints'])

            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['agent_cost'] = get_agent_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        if conflict_graph:
            root['h_val'] = compute_cg_heuristic(self.my_map, self.starts, self.goals, self.num_of_agents, root)
        self.push_node(root, conflict_graph)

        while len(self.open_list) > 0:
            # Get best node N from OPEN (lowest solution cost)
            next_node = self.pop_node(conflict_graph)
            # If N has no conflicts then return solution (N is goal)
            if len(next_node['collisions']) == 0:
                self.print_results(next_node)
                return next_node['paths']
            # Iterate over all collisions
            for conflict in next_node['collisions']:
                # Build MDD for each agent
                a1 = conflict['a1']
                a2 = conflict['a2']
                mdd1 = build_mdd(self.my_map, self.starts[a1], self.goals[a1], a1, next_node['constraints'], next_node['agent_cost'][a1])
                mdd2 = build_mdd(self.my_map, self.starts[a2], self.goals[a2], a2, next_node['constraints'], next_node['agent_cost'][a2])

                # Check for cardinal/semi-cardinal conflict
                cardinal_conflict = detect_cardinal_conflict(mdd1, mdd2)
            
            if cardinal_conflict is not None:
                collision = cardinal_conflict
            else:
                # If no cardinal/semi-cardinal conflict exists, arbitrarily choose a non-cardinal conflict
                collision = next_node['collisions'][0] # Get first conflict
            constraints = standard_splitting(collision)  # disjoint_splitting(collision)

            for constraint in constraints:
                new_child_node = dict()
                new_child_node['constraints'] = [constraint] + next_node['constraints']
                new_child_node['paths'] = [] + next_node['paths']

                agent_in_constraint = constraint['agent']
                # Update child solution by invoking low level search
                path = a_star(self.my_map,
                              self.starts[agent_in_constraint],
                              self.goals[agent_in_constraint],
                              self.heuristics[agent_in_constraint],
                              agent_in_constraint,
                              new_child_node['constraints'])

                if path is not None and len(path) > 0:
                    new_child_node['paths'][agent_in_constraint] = [] + path
                    new_child_node['collisions'] = detect_collisions(new_child_node['paths'])
                    new_child_node['cost'] = get_sum_of_cost(new_child_node['paths'])
                    new_child_node['agent_cost'] = get_agent_cost(new_child_node['paths'])
                    if conflict_graph:
                        new_child_node['h_val'] = compute_cg_heuristic(self.my_map, self.starts, self.goals, self.num_of_agents, new_child_node)
                    self.push_node(new_child_node, conflict_graph)

        self.print_results(root)
        return None

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        process = psutil.Process(os.getpid())
        print("Memory usage:    {} mb".format(process.memory_info().rss / 1000000))
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
