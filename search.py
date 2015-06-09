# Christopher Oliver, CS5100, oliverc, oliver.ch@husky.neu.edu
# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for 
# educational purposes provided that (1) you do not distribute or publish 
# solutions, (2) you retain this notice, and (3) you provide clear 
# attribution to UC Berkeley, including a link to 
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
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
    "*** YOUR CODE HERE ***"
    # Define some global variables and objects used by the recursive search
    # function.
    
    # Structure:
    #   key - state, for example (5,5)
    #   list
    #       [0] - action taken to reach key state
    #       [1] - parent state from which action was taken
    global dfsExplored
    dfsExplored = {}

    # Stack object used to store state successors with a LIFO queuing policy.
    global dfsStack
    dfsStack = util.Stack()

    # Boolean set to False when a goal state is reached.
    global dfsSearchInProgress
    dfsSearchInProgress = True

    # Variable holding problem goal state once found, used as a starting
    # point to build the returned list of actions after the search algorithm
    # completes.
    global dfsSearchGoal
 
    # Build the dfsExplored dictionary which captures all of the information
    # gleaned from the search algorithm.
    recursivedepthFirstSearch((problem.getStartState(),' ',0), problem, 'root')

    # If the search reached it's goal build a list of actions using the
    # dfsExplored dictionary, else return an empty list.
    if dfsSearchInProgress:
        emptyList = []
        return emptyList
    else:
        return buildSearchSolution(dfsSearchGoal, dfsExplored)


def recursivedepthFirstSearch(vertex, problem, parent):
    global dfsSearchInProgress
    global dfsSearchGoal
    # Add this vertex to the explored dictionary as it will be expanded.
    dfsExplored[vertex[0]] = [vertex[1],parent]
    # Check if this is the goal vertex, if so set globals and return.
    if problem.isGoalState(vertex[0]):
        dfsSearchInProgress = False
        dfsSearchGoal = vertex[0]
        return 0
    # Find the vertex's successors, adding them to the stack.
    for successors in problem.getSuccessors(vertex[0]):
        dfsStack.push((successors, vertex[0]))
    # Pop from the stack with the following considerations:
    # - stack must have vertices in it
    # - ignore vertices that have been previously explored
    while dfsSearchInProgress:
        if dfsStack.isEmpty():
            break
        else:
            nextVertex, nextParent = dfsStack.pop()
            if nextVertex[0] in dfsExplored:
                continue
            else:
                recursivedepthFirstSearch(nextVertex, problem, nextParent)
                break
    return 0


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # Obtain the problem start state.
    startState = problem.getStartState()
    
    # Test to see if the problem start state is also the problem goal state.
    # If so return an empty list of actions.
    if problem.isGoalState(startState):
        return []

    # Queue object used to store state successors with a FIFO queuing policy.
    bfsFrontier = util.Queue()

    # Add the initial problem state to the frontier.
    bfsFrontier.push(startState)
    
    # Set used to track which states have already been explored, initially
    # empty.
    exploredStates = []
    setOfExploredStates = set(exploredStates)

    # Build a dictionary to keep track of the parent node for each node and the
    # action taken to reach this node from their parent.  This will be used to
    # create the list of actions ouput by bfs.
    #   key - state, for example (5,5)
    #   list
    #       [0] - action taken to reach key state
    #       [1] - parent state from which action was taken
    stateDictionary = {}

    # Add the start state to the dictionary with a parent of 'root' and an
    # action of 'none'.
    stateDictionary[startState] = ('none', 'root')

    while True:
        if bfsFrontier.isEmpty():
            return []
        bfsState = bfsFrontier.pop()
        if bfsState not in setOfExploredStates:
            if problem.isGoalState(bfsState):
                return buildSearchSolution(bfsState, stateDictionary)
            setOfExploredStates.add(bfsState)
            for successors in problem.getSuccessors(bfsState):
                if successors[0] not in setOfExploredStates:
                    bfsFrontier.push(successors[0])
                    if successors[0] not in stateDictionary:
                        stateDictionary[successors[0]] = (successors[1]
                                                          ,bfsState)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # Obtain the problem start state.
    startState = problem.getStartState()
    
    # Build a Priority Queue which will use path cost for order, low to high.
    ucsFrontier = util.PriorityQueue()
    
    # Add the initial state to the Priority Queue with an initial path cost of
    # 0.
    ucsFrontier.push(startState, 0)
 
    # Set used to track which states have already been explored, initially
    # empty.
    exploredStates = []
    setOfExploredStates = set(exploredStates)
    
    # Build a dictionary to keep track of the parent node for each node, the
    # action taken to reach this node from their parent and the cost.  This will
    # be used to create the list of actions ouput by ucs.
    #   key - state, for example (5,5)
    #   list
    #       [0] - action taken to reach key state
    #       [1] - parent state from which action was taken
    #       [2] - cost to this node through this parent
    stateDictionary = {}
    
    # Add the start state to the dictionary with a parent of 'root' an action
    # of 'none' and a path cost of 0.
    stateDictionary[startState] = ('none', 'root', 0)
    
    while True:
        if ucsFrontier.isEmpty():
            return []
        ucsState = ucsFrontier.pop()
        # This test is necessary because the priority queue is implemented so
        # that the same state may appear more than once if it has more than one
        # priority.
        if ucsState not in setOfExploredStates:
            if problem.isGoalState(ucsState):
                return buildSearchSolution(ucsState, stateDictionary)
            setOfExploredStates.add(ucsState)
            for successors in problem.getSuccessors(ucsState):
                successorsCost = stateDictionary[ucsState][2] + successors[2]
                if successors[0] not in setOfExploredStates:
                    ucsFrontier.push(successors[0], successorsCost)
                if successors[0] not in stateDictionary:
                    stateDictionary[successors[0]] = [successors[1],
                                                      ucsState,
                                                      successorsCost]
                else:
                    if stateDictionary[successors[0]][2] > successorsCost:
                        stateDictionary[successors[0]] = [successors[1],
                                                          ucsState,
                                                          successorsCost]


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    # Obtain the problem start state.
    startState = problem.getStartState()
    
    # Build a Priority Queue which will use path cost for order, low to high.
    astarFrontier = util.PriorityQueue()
    
    # Add the initial state to the Priority Queue with an initial path cost of
    # 0.  Set the parent node to root and action to none, this is used in
    # buildSearchSolution.
    astarFrontier.push(startState, heuristic(startState,problem))
    
    # Set used to track which states have already been explored, initially
    # empty.
    exploredStates = []
    setOfExploredStates = set(exploredStates)
    
    # Build a dictionary to keep track of the parent node for each node, the
    # action taken to reach this node from their parent and the cost.  This will
    # be used to create the list of actions ouput by ucs.
    #   key - state, for example (5,5)
    #   list
    #       [0] - action taken to reach key state
    #       [1] - parent state from which action was taken
    #       [2] - cost to this node through this parent
    stateDictionary = {}
    
    # Add the start state to the dictionary with a parent of 'root' an action
    # of 'none' and a path cost of 0.
    stateDictionary[startState] = ('none', 'root', 0)
    
    while True:
        if astarFrontier.isEmpty():
            return []
        astarState = astarFrontier.pop()
        # This test is necessary because the priority queue is implemented so
        # that the same state may appear more than once if it has more than one
        # priority.
        if astarState not in setOfExploredStates:
            if problem.isGoalState(astarState):
                return buildSearchSolution(astarState, stateDictionary)
            setOfExploredStates.add(astarState)
            for successors in problem.getSuccessors(astarState):
                successorsCost = stateDictionary[astarState][2] + successors[2]
                if successors[0] not in setOfExploredStates:
                    astarFrontier.push(successors[0],
                                       successorsCost + heuristic(successors[0],
                                                                  problem))
                if successors[0] not in stateDictionary:
                    stateDictionary[successors[0]] = [successors[1],
                                                      astarState,
                                                      successorsCost]
                else:
                    if stateDictionary[successors[0]][2] > successorsCost:
                        stateDictionary[successors[0]] = [successors[1],
                                                          astarState,
                                                          successorsCost]


# Recursively build a list of actions to reach the given search state using
# the explored dictionary.  The code traverses a linked list from the goal
# state back to the root state using parents that were captured during
# the search algorithm execution.  Root is found at the deepest recursion
# depth, the list of actions from root to goal is built on the way back from
# deepest recursion depth starting with the empty list.
def buildSearchSolution(searchState,searchDictionary):
    if searchDictionary[searchState][1] == "root":
        return []
    else:
        updatedList = buildSearchSolution(searchDictionary[searchState][1],
                                          searchDictionary)
        updatedList.append(searchDictionary[searchState][0])
        return updatedList


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
