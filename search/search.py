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
    """

    # print "Start:", problem.getStartState()
    #print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    # print "Start's successors:", problem.getSuccessors(problem.getStartState())


    "*** YOUR CODE HERE ***"

##stack created for dfs
    dfs_st=util.Stack()
    dfs_st.push((problem.getStartState(), [], 0))
    ##visited and action_state array to keep track of visited nodes and the actions (N/S/E/W) where the pacman goes
    visited = []
    action_state = []

    ## loop through all states while the stack is empty
    while (not dfs_st.isEmpty()):
        ## pop first element of the stack
        node = dfs_st.pop()
        ##action states of the popped node
        action_state = node[1]

        ## return is goal state is reached
        if (problem.isGoalState(node[0])):
            return action_state
        else:
            ## check all successor nodes for the given node and append not visited nodes to the queue
            ## Also mark all the successor nodes being added to the stack as visited
            if (not node[0] in visited):
                visited.append(node[0])
                for sucnode in problem.getSuccessors(node[0]):
                    dfs_st.push((sucnode[0], action_state + [sucnode[1]], sucnode[2]))

    #util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
##QUEUE created for bfs
    bfs_st = util.Queue()
    bfs_st.push((problem.getStartState(), [], 0))
    ##visited and action_state array to keep track of visited nodes and the actions (N/S/E/W) where the pacman goes
    visited = []
    action_state = []

    ## loop through all states while the queue is empty
    while (not bfs_st.isEmpty()):
        ## pop first element of the queue
        node = bfs_st.pop()
        ##action states of the popped node are stored
        action_state = node[1]

        ## return is goal state is reached
        if (problem.isGoalState(node[0])):
            return action_state
        else:
            ## check all successor nodes for the given node and append not visited nodes to the queue
            if (not node[0] in visited):
                visited.append(node[0])
                for sucnode in problem.getSuccessors(node[0]):
                        bfs_st.push((sucnode[0], action_state + [sucnode[1]], sucnode[2]))
    #util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
## Priority queue to store uniform cost search nodes
    bfs_st = util.PriorityQueue()
    bfs_st.push((problem.getStartState(), [], 0),0)
    ##visited and action_state array to keep track of visited nodes and the actions (N/S/E/W) where the pacman goes
    visited = []
    action_state = []

    ## loop through all states while the queue is empty
    while (not bfs_st.isEmpty()):
        ## pop first element of the queue
        node = bfs_st.pop()
        ##action states of the popped node
        action_state = node[1]
        cost=node[2]

        ## return is goal state is reached
        if (problem.isGoalState(node[0])):

            return action_state
        else:
            ## check all successor nodes for the given node and append not visited nodes to the queue
            if (not node[0] in visited):
                visited.append(node[0])
                for sucnode in problem.getSuccessors(node[0]):
                ## add the cost of previous nodes/path to the current path value, set is as the priority factor
                ## of the priority queue
                        bfs_st.push((sucnode[0], action_state + [sucnode[1]], cost+sucnode[2]),cost+sucnode[2])


    #util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    ## Priority queue to store costs for A* search
    bfs_st = util.PriorityQueue()

    bfs_st.push((problem.getStartState(), [], 0), 0)
    ##visited and action_state array to keep track of visited nodes and the actions (N/S/E/W) where the pacman goes
    visited = []
    #hash of current state : corners: write hash function : make object
    action_state = []

    ## loop through all states while the queue is empty
    while (not bfs_st.isEmpty()):
        ## pop first element of the queue
        node = bfs_st.pop()
        ##action states of the popped node
        action_state = node[1]
        cost = node[2]
        ## append the visited node to be false

        ## return is goal state is reached
        if (problem.isGoalState(node[0])):

            return action_state
        else:
            ## check all successor nodes for the given node and append not visited nodes to the queue
            if (not node[0] in visited):
                visited.append(node[0])
                for sucnode in problem.getSuccessors(node[0]):
                ## Call the heuristic funcion which calculates distance by the manhattan distance, add it
                ## to the cost obtained earlier. This gives the priority with which the queue is ordered
                    pos=heuristic(sucnode[0],problem)
                    bfs_st.push((sucnode[0], action_state + [sucnode[1]], cost + sucnode[2]), cost + sucnode[2]+pos)


                    #util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
