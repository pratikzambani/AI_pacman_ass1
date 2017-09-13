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
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """

    start_state = problem.getStartState()
    stack = util.Stack()
    explored_list = []
    stack.push((start_state,[]))
    while 1:
        if stack.isEmpty():
            return []
        node, actionsToNode = stack.pop()
        if problem.isGoalState(node):
            return actionsToNode
        explored_list.append(node)
        for s in problem.getSuccessors(node):
            if s[0] not in explored_list:
                action=s[1]
                stack.push((s[0],actionsToNode + [action]))

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    start_state = problem.getStartState()
    q = util.Queue()
    q.push((start_state, []))
    explored_list = []
    while 1:
        if q.isEmpty():
            return []
        node, actionsToNode = q.pop()
        if problem.isGoalState(node):
            return actionsToNode
        if node in explored_list:
            continue
        else:
            explored_list.append(node)
        for s in problem.getSuccessors(node):
            if s[0] not in explored_list:
                action = s[1]
                q.push((s[0], actionsToNode + [action]))

def uniformCostSearch(problem):
    """Search the node of least total cost first."""

    """ Need to check if update needs to be explicitly called"""
    start_state = problem.getStartState()
    q = util.PriorityQueue()
    q.push((start_state, []), -1)

    explored_list = []
    while 1:
        if q.isEmpty():
            return []
        node, actionsToNode = q.pop()
        if problem.isGoalState(node):
            return actionsToNode
        if node in explored_list:
            continue
        else:
            explored_list.append(node)
        for s in problem.getSuccessors(node):
            if s[0] not in explored_list:
                action = s[1]
                q.push((s[0], actionsToNode + [action]), s[2])

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    start_state = problem.getStartState()
    q = util.PriorityQueue()
    q.push((start_state,0), 0)

    explored_list = []
    parent_map = {start_state: (-1, -1)}
    while 1:
        if q.isEmpty():
            return []
        node = q.pop()
        nodeState=node[0]
        if problem.isGoalState(nodeState):
            actions = []
            while parent_map[nodeState][1] != -1:
                actions.append(parent_map[nodeState][1])
                nodeState = parent_map[nodeState][0]
            ractions = list(reversed(actions))
            return ractions
        explored_list.append(node)

        for s in problem.getSuccessors(nodeState):
            if s[0] not in parent_map and s[0] not in explored_list:
                    costToNode = node[1] + s[2]
                    q.push((s[0],costToNode), costToNode + heuristic(s[0], problem))
                    parent_map[s[0]] = (nodeState, s[1])

    # start_state = problem.getStartState()
    # q = util.PriorityQueue()
    # q.push((start_state, 0, []), 0)
    #
    # explored_list = []
    # while 1:
    #     if q.isEmpty():
    #         return []
    #     node = q.pop()
    #     nodeState = node[0]
    #     actionsToNode = node[2]
    #     if node in explored_list:
    #         continue
    #     else:
    #         explored_list.append(node)
    #     if problem.isGoalState(nodeState):
    #         return actionsToNode
    #
    #     for s in problem.getSuccessors(nodeState):
    #         if s[0] not in explored_list:
    #             costToNode = node[1] + s[2]
    #             action = s[1]
    #             q.push((s[0], costToNode, actionsToNode + [action]), costToNode + heuristic(s[0], problem))


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
