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

def g_cost(node):
    return node[2]

def f_cost(node):
    return node[2] + node[3]


def search_generic(problem, fringe, heuristic):
    """
    Generic Search algorithm - can be adapted to
    DFS, BFS, UCS, A*
    :param problem: Search Problem
    :param fringe: Data structure used to hold fringe nodes
    :param heuristic: Heuristic function
    :return: Path (array of actions) from start to goal
    """
    # Init search
    # start state represents the state space
    # representation derived from the problem
    # For simple path problems this start space is just the
    # position.
    # For more complex problems - like find corers
    # this start space may include more information like
    # the corners visited
    start_state = problem.getStartState()
    # search node representation:
    # (state, path to state, g, h)
    start_node = (start_state, [], 0, heuristic(start_state, problem))
    fringe.push(start_node)
    # Closed set - The set of states already explored
    closed_set = []

    while not fringe.isEmpty():
        current_node = fringe.pop()
        # current_node[0] represents the state contained
        # in the node - does not include path information
        # Do not explore states already in the closed set
        if problem.isGoalState(current_node[0]):
            return current_node[1]
        if current_node[0] in closed_set:
            pass
        else:
            closed_set.append(current_node[0])
            for successor in problem.getSuccessors(current_node[0]):
                # Successor tuple contains
                # (next state, action to next state, cost)
                next_state = successor[0]
                if next_state in closed_set:
                    pass
                else:
                    # construct path to next node
                    path = []
                    for i in current_node[1]:
                        path.append(i)
                    path.append(successor[1])
                    cost = current_node[2] + successor[2]
                    fringe.push((next_state, path, cost, heuristic(next_state, problem)))
    # Failure
    return None

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    return search_generic(problem, util.Stack(), nullHeuristic)

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    return search_generic(problem, util.Queue(), nullHeuristic)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    return search_generic(problem, util.PriorityQueueWithFunction(g_cost), nullHeuristic)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    return search_generic(problem, util.PriorityQueueWithFunction(f_cost), heuristic)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
