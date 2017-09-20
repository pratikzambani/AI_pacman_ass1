# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest node in the tree first.
    Algorithm uses Stack data structure to achieve LIFO traversal.
    During traversal, an explored_list is maintained to keep track of already visited states.
    Actions to reach any node are also pushed along with state positions onto stack as a tuple

    When goal state is encountered, the actions accumulated with the state is returned
    """
    start_state = problem.getStartState()
    """Using stack to achieve LIFO traversal"""
    stack = util.Stack()
    explored_list = []
    """Pushing start state and an empty list of actions initially"""
    stack.push((start_state,[]))
    while 1:
        if stack.isEmpty():
            return []
        # Pop the state position and the list of actions to that state
        node, actions_to_node = stack.pop()
        if problem.isGoalState(node):
            return actions_to_node
        # If node is already explored, no action, continue to the next element to be popped
        if node in explored_list:
            continue
        explored_list.append(node)
        for successor in problem.getSuccessors(node):
            successor_state = successor[0]
            action_to_successor = successor[1]
            if successor_state not in explored_list:
                 # Append acton to reach successor to the list of actions to reach parent and push(state, actions)
                stack.push((successor_state, actions_to_node + [action_to_successor]))

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first.
       Algorithm uses Queue data structure to achieve FIFO traversal.
       During traversal, an explored_list is maintained to keep track of already visited states.
       Actions to reach any node are also pushed along with state positions onto queue as a tuple

       When goal state is encountered, the actions accumulated with the state is returned
       """
    start_state = problem.getStartState()
    # Using queue to achieve FIFO traversal
    q = util.Queue()
    # Pushing start state and an empty list of actions initially
    q.push((start_state, []))
    explored_list = []
    while 1:
        if q.isEmpty():
            return []
        # Pop the state position and the list of actions to that state
        node, actions_to_node = q.pop()
        if problem.isGoalState(node):
            return actions_to_node
        # If node is already explored, no action, continue to the next element to be popped
        if node in explored_list:
            continue
        else:
            explored_list.append(node)
        for successor in problem.getSuccessors(node):
            successor_state = successor[0]
            action_to_successor = successor[1]
            if successor_state not in explored_list:
                # Append acton to reach successor to the list of actions to reach parent and push(state, actions)
                q.push((successor_state, actions_to_node + [action_to_successor]))

def uniformCostSearch(problem):
    """Search the node of least total cost first.
        Algorithm uses Priority Queue data structure to achieve traversal with minimum cost nodes first.
        During traversal, an explored_list is maintained to keep track of already visited states.
        With actions and state, cost to reach any node is also pushed along with state positions onto queue as a tuple
        Priority pushed into queue is the cost to reach that node

        When goal state is encountered, the actions accumulated with the state is returned
        """
    start_state = problem.getStartState()
    # Using Priority queue to achieve highest priority node(least cost node) traversal
    q = util.PriorityQueue()
    # Pushing start state and an empty list of actions, 0 cost, and 0 as priority initially
    q.push((start_state, [], 0), 0)

    explored_list = []
    while 1:
        if q.isEmpty():
            return []
        # Pop the state position, list of actions to that state and cost to reach that node
        node_state, actions_to_node, cost_to_node = q.pop()
        if problem.isGoalState(node_state):
            return actions_to_node
        # If node is already explored, no action, continue to the next element to be popped
        if node_state in explored_list:
            continue
        else:
            explored_list.append(node_state)
        for successor in problem.getSuccessors(node_state):
            successor_state = successor[0]
            action_to_successor = successor[1]
            cost_to_successor = successor[2]
            if successor_state not in explored_list:
                # Append acton to reach successor to the list of actions to reach parent
                # Add cost to successor to the cost to reach parent
                # push (state, actions, total cost) on queue with Priority as total cost
                q.push((successor_state, actions_to_node + [action_to_successor], cost_to_node + cost_to_successor),
                       cost_to_node + cost_to_successor)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first.
        Algorithm uses Priority Queue data structure and heuristic function to achieve traversal with minimum cost nodes first.
        During traversal, an explored_list is maintained to keep track of already visited states.
        With actions and state, cost to reach any node is also pushed along with state positions onto queue as a tuple
        Priority pushed into queue is the cost to reach that node and a heauristic function used to estimate cost to goal

        When goal state is encountered, the actions accumulated with the state is returned
        """
    start_state = problem.getStartState()
    # Using Priority queue to achieve highest priority node(least cost node) traversal
    q = util.PriorityQueue()
    # Pushing start state and an empty list of actions, 0 cost, and 0 as priority initially
    q.push((start_state,[], 0 ), 0)

    explored_list = []
    while 1:
        if q.isEmpty():
            return []
        # Pop the state position, list of actions to that state and cost to reach that node
        node_state, actions_to_node, cost_to_node = q.pop()
        # If node is already explored, no action, continue to the next element to be popped
        if node_state in explored_list:
            continue
        else:
            explored_list.append(node_state)
        if problem.isGoalState(node_state):
            return actions_to_node
        for successor in problem.getSuccessors(node_state):
            successor_state = successor[0]
            action_to_successor = successor[1]
            cost_to_successor = successor[2]
            if successor_state not in explored_list:
                # Append acton to reach successor to the list of actions to reach parent
                # Add cost to successor to the cost to reach parent
                # push (state,action, total_cost) on queue with Priority as total cost + heuristic estimate from state to goal
                cost_from_start = cost_to_node + cost_to_successor
                q.push((successor_state, actions_to_node + [action_to_successor], cost_from_start), cost_from_start + heuristic(successor_state, problem))


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
