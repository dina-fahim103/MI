from problem import HeuristicFunction, Problem, S, A, Solution
from collections import deque
from helpers import utils

import heapq

# all code is written& documented solely by me
# backtracking function for returning the correct path fro initial to goalstate
# appending the last action that led me to goal then taking it's parent state and appending the action that led me to it
# then taking it's parent and so on finally when I reach the initial state I do nothing reverse the path and return it
# the function takes two dictionaries one for parent states and other for previous action that led to state and takes
# the initial state and goal state


def backtracking(parent, preaction, initialstate, goalstate):
    lastaction = preaction[goalstate]
    path = [lastaction]
    traversenode = parent[goalstate]
    while traversenode != initialstate:
        path.append(preaction[traversenode])
        traversenode = parent[traversenode]
    path.reverse()
    return path


# I used some(open source) code from internet to help implement the datastructrues, didn't implement it from scratch as there's no need to(not required)
# abstract datastructure to inherit the datastructures needed in the project
# queue datastructure to implement the bradth first search algorithm
class Queue():

    def __init__(self):
        self._queue = deque()

    def pop(self) -> S:
        return self._queue.popleft()

    def push(self, state):
        return self._queue.append(state)

    def empty(self) -> bool:
        return not bool(self._queue)

    def notempty(self) -> bool:
        return not self.empty()


# the bradth first search algorithm works by a queue to keep in the traversing states in it, the visted set to make sure
# I don't explore the same state more than once, the preaction dictionary to save the action that led me to the state I am in
# first I push the initial state into queue then as long as queue not empty I pop a state and if it's not visited I add it to visited
# set the I check if I reached a goal if yes I a call the backtrack function explained above to return me the path, if no I call the
# get actions method and for every possible action I have I call getsuccessor function to give e the next state of every possible action
# then I check if next state is not expanded I store the parent state and the previos action for evevry next state finally I push it into
# queue If queue is epty without reaching a goal the path is None and I retuurn it
def BreadthFirstSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    parent = {}
    preaction = {}
    visited = set()
    expanded = set()
    ds = Queue()
    ds.push(initial_state)

    while ds.notempty():
        state = ds.pop()
        if state not in visited:

            visited.add(state)
            if problem.is_goal(state):
                return backtracking(parent, preaction, initial_state, state)

            for action in problem.get_actions(state):
                nextstate = problem.get_successor(state, action)
                if nextstate not in expanded:
                    parent[nextstate] = state
                    preaction[nextstate] = action
                    ds.push(nextstate)
                    expanded.add(nextstate)
    return None


# stack data structure to implement the depth first search algorith
class Stack():
    def __init__(self):
        self._stack = []

    def pop(self) -> S:
        return self._stack.pop()

    def push(self, state):
        self._stack.append(state)

    def empty(self) -> bool:
        return len(self._stack) == 0

    def notempty(self) -> bool:
        return not self.empty()


# the depth first search algorithm works by a stack to keep in the traversing states in it, the visted set to make sure
# I don't explore the same state more than once, the preaction dictionary to save the action that led me to the state I am in
# first I push the initial state into stack then as long as stack not empty I pop a state and if it's not visited I add it to visited
# set the I check if I reached a goal if yes I a call the backtrack function explained above to return me the path, if no I call the
# get actions method and for every possible action I have I call getsuccessor function to give e the next state of every possible action
# then I check if next state is not visited I store the parent state and the previos action for evevry next state finally I push it into
# stack If stack is epty without reaching a goal the path is None and I retuurn it
def DepthFirstSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    parent = {}
    preaction = {}
    visited = set()
    ds = Stack()
    ds.push(initial_state)

    while ds.notempty():
        state = ds.pop()
        if state not in visited:

            visited.add(state)
            if problem.is_goal(state):
                return backtracking(parent, preaction, initial_state, state)

            for action in problem.get_actions(state):
                nextstate = problem.get_successor(state, action)
                if nextstate not in visited:
                    parent[nextstate] = state
                    preaction[nextstate] = action
                    ds.push(nextstate)

    return None


# priority queue datastructure to implement the uniform cost, A* and greedy best first search algorithms
# used the heapq python built in module to implement the datastructure
class PQueue():

    def __init__(self):
        self.heap = []
        self.count = 0

    def push(self, state, priority):
        entry = (priority, self.count, state)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (_, _, state) = heapq.heappop(self.heap)
        return state

    def empty(self):
        return len(self.heap) == 0

    def notempty(self) -> bool:
        return not self.empty()


# the uniform cost search algorithm works by a priority queue to keep in the traversing states in it by their heuristic, the visted set
#  to make sure I don't explore the same state more than once, the preaction dictionary to save the action that led me to the state I am in
# first I push the initial state into priority queue then as long as priority queue not empty I pop a state and if it's not visited I add it to visited
# set the I check if I reached a goal if yes I a call the backtrack function explained above to return me the path, if no I call the
# get actions method and for every possible action I have I call getsuccessor function to give e the next state of every possible action
# then I check if next state is not expanded I store the parent state and the previos action for evevry next state and I store the cost
# finally I push it into priority queue If priority queue is empty without reaching a goal the path is None and I retuurn it
def UniformCostSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    parent = {}
    preaction = {}
    cost = {}
    visited = set()
    expanded = set()
    ds = PQueue()
    ds.push(initial_state, 0.0)
    while ds.notempty():
        state = ds.pop()

        if state not in visited:
            visited.add(state)
            # found a bug in special cases when I reach a goal but it's cost it's not the least
            # I should continue with the lowest cost until I reach the goal normally if it exists
            # a pesudo code for bug solution but it's not working yet
            # cmpcost = []
            #cmpcostdict = {}
            #       if problem.is_goal(state):
            #          while ds.notempty():
            #              traverse = ds.pop()
            #              cmpcost.append(cost[traverse])
            #              cmpcostdict[traverse] = cost[traverse]
            #          if min(cmpcost) > cost[state]:
            #              return backtracking(parent, preaction, initial_state, state)
            #        else:
            #              for i in cmpcostdict:
            #                 ds.push(i, cmpcostdict[i])
            #         return backtracking(parent, preaction, initial_state, state)

            if problem.is_goal(state):
                return backtracking(parent, preaction, initial_state, state)

            for action in problem.get_actions(state):
                nextstate = problem.get_successor(state, action)
                if nextstate not in expanded:
                    cost[action] = problem.get_cost(initial_state, action)
                    parent[nextstate] = state
                    preaction[nextstate] = action
                    ds.push(nextstate, cost[action])
                    expanded.add(nextstate)
    return None


# the A* search algorithm works by a priority queue to keep in the traversing states in it by their heuristic, the visted set
#  to make sure I don't explore the same state more than once, the preaction dictionary to save the action that led me to the state I am in
# first I push the initial state into priority queue then as long as priority queue not empty I pop a state and if it's not visited I add it to visited
# set the I check if I reached a goal if yes I a call the backtrack function explained above to return me the path, if no I call the
# get actions method and for every possible action I have I call getsuccessor function to give e the next state of every possible action
# then I check if next state is not expanded I store the parent state and the previos action for evevry next state and I store the cost+heuristic
# finally I push it into priority queue If priority queue is empty without reaching a goal the path is None and I retuurn it
def AStarSearch(problem: Problem[S, A], initial_state: S, heuristic: HeuristicFunction) -> Solution:
    parent = {}
    preaction = {}
    cost = {}
    heuris = {}
    visited = set()
    expanded = set()
    ds = PQueue()
    ds.push(initial_state, 0.0)

    while ds.notempty():
        state = ds.pop()

        if state not in visited:
            visited.add(state)
            # a solution to the bug above in uniform cost search will be the same solution the bug here because it's the same thing
            if problem.is_goal(state):
                return backtracking(parent, preaction, initial_state, state)

            for action in problem.get_actions(state):
                nextstate = problem.get_successor(state, action)
                if nextstate not in expanded:
                    cost[action] = problem.get_cost(initial_state, action)
                    heuris[nextstate] = heuristic(problem, nextstate)
                    parent[nextstate] = state
                    preaction[nextstate] = action
                    ds.push(nextstate, cost[action]+heuris[nextstate])
                    expanded.add(nextstate)
    return None


# the greedy best first search algorithm works by a priority queue to keep in the traversing states in it by their heuristic, the visted set
#  to make sure I don't explore the same state more than once, the preaction dictionary to save the action that led me to the state I am in
# first I push the initial state into priprity queue then as long as priority queue not empty I pop a state and if it's not visited I add it to visited
# set the I check if I reached a goal if yes I a call the backtrack function explained above to return me the path, if no I call the
# get actions method and for every possible action I have I call getsuccessor function to give e the next state of every possible action
# then I check if next state is not expanded I store the parent state and the previos action for evevry next state and I store the heiristic
# finally I push it into priority queue If priority queue is empty without reaching a goal the path is None and I retuurn it
def BestFirstSearch(problem: Problem[S, A], initial_state: S, heuristic: HeuristicFunction) -> Solution:
    parent = {}
    preaction = {}
    heuris = {}
    visited = set()
    expanded = set()
    ds = PQueue()
    ds.push(initial_state, 0.0)

    while ds.notempty():
        state = ds.pop()

        if state not in visited:
            visited.add(state)
            if problem.is_goal(state):
                return backtracking(parent, preaction, initial_state, state)

            for action in problem.get_actions(state):
                nextstate = problem.get_successor(state, action)
                if nextstate not in expanded:
                    heuris[nextstate] = heuristic(problem, nextstate)
                    parent[nextstate] = state
                    preaction[nextstate] = action
                    ds.push(nextstate, heuris[nextstate])
                    expanded.add(nextstate)

    return None
