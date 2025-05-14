import problem 
from node import Node
from priority_queue import PriorityQueue
import time

class SearchTimeOutError(Exception):
    pass

def compute_g(algorithm, node, goal_state):
    """
        Evaluates the g() value.

        Parameters
        ===========
            algorithm: str
                The algorithm type based on which the g-value will be computed.
            node: Node
                The node whose g-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The g-value for the node.
    """

    if algorithm == "bfs":
        return node.get_depth()

    if algorithm == "astar":

        return node.get_total_action_cost()

    elif algorithm == "gbfs":

        return 0

    elif algorithm == "ucs":

        return node.get_total_action_cost()

    elif algorithm == "custom-astar":

       return NotImplementedError()

    # Should never reach here.
    assert False
    return float("inf")


def compute_h(algorithm, node, goal_state):
    """
        Evaluates the h() value.

        Parameters
        ===========
            algorithm: str
                The algorithm type based on which the h-value will be computed.
            node: Node
                The node whose h-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The h-value for the node.
    """

    if algorithm == "bfs":
        
        return 0

    if algorithm == "astar":

        return get_manhattan_distance(node.get_state(), goal_state)

    elif algorithm == "gbfs":

        return get_manhattan_distance(node.get_state(), goal_state)

    elif algorithm == "ucs":
        
        return 0

    elif algorithm == "custom-astar":

        return NotImplementedError()

    # Should never reach here.
    assert False
    return float("inf")


def get_manhattan_distance(from_state, to_state):
    return abs(from_state.x - to_state.x) + abs(from_state.y - to_state.y)


def get_custom_heuristic(from_state, to_state):
    
    return NotImplementedError()

def graph_search(algorithm, time_limit):
    """
        Performs a search using the specified algorithm.
        
        Parameters
        ===========
            algorithm: str
                The algorithm to be used for searching.
            time_limit: int
                The time limit in seconds to run this method for.
                
        Returns
        ========
            tuple(list, int)
                A tuple of the action_list and the total number of nodes
                expanded.
    """
    
    # The helper allows us to access the problem functions.
    helper = problem.Helper()
    
    # Get the initial and the goal states.
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()[0]

    #Initialize the init node of the search tree and compute its f_score
    init_node = Node(init_state, None, 0, None, 0)
    f_score = compute_g(algorithm, init_node, goal_state) \
        + compute_h(algorithm, init_node, goal_state)

       
    # Initialize the fringe as a priority queue.
    priority_queue = PriorityQueue()
    priority_queue.push(f_score, init_node)
    

    # action_list should contain the sequence of actions to execute to reach from init_state to goal_state
    action_list = []

    # total_nodes_expanded maintains the total number of nodes expanded during the search
    total_nodes_expanded = 0
    time_limit = time.time() + time_limit
   
    explored_states = set()
    
    """
    Implementation for graph-based search.

    The function must determine the optimal sequence of actions (action_list)
    to transition from the start state to the goal state, while also tracking 
    the total number of expanded nodes.
    """

    while not priority_queue.is_empty():
        if time.time() >= time_limit:
            raise SearchTimeOutError(f"Search timed out after {time_limit} seconds.")

        curr_node = priority_queue.pop()
        state = curr_node.get_state()

        if state not in explored_states and state.x != -1 and state.y != -1:
            explored_states.add(state)
       
            # If the goal is reached, trace back the solution path
            if helper.is_goal_state(state):
                while curr_node.get_parent():
                    action_list.append(curr_node.get_action())
                    curr_node = curr_node.get_parent()
                action_list.reverse()
                return action_list, total_nodes_expanded

            # Expand the current state to generate successors
            # Ensure the state is valid and not previously explored
            next_moves = helper.get_successor(state)
            total_nodes_expanded += 1

            # Iterate through successor states and add them to the priority queue
            for action, (new_state, move_cost) in next_moves.items():
                child_node = Node(new_state, curr_node, curr_node.get_depth() + 1, action, move_cost)
                f_value = compute_g(algorithm, child_node, goal_state) + compute_h(algorithm, child_node, goal_state)
                priority_queue.push(f_value, child_node)


    return action_list, total_nodes_expanded