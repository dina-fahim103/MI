from dungeon import DungeonProblem, DungeonState
from mathutils import Direction, Point, euclidean_distance, manhattan_distance
from helpers import utils


def weak_heuristic(problem: DungeonProblem, state: DungeonState):
    return euclidean_distance(state.player, problem.layout.exit)


# I'm trying to compute the euclidean_distance, manhattan_distance and get their average and make
# some computations to implement the strong heuristic function
def strong_heuristic(problem: DungeonProblem, state: DungeonState) -> float:
    # TODO: not yet finished some computations are missing
    euclidean = euclidean_distance(state.player, problem.layout.exit)
    manhattan = manhattan_distance(state.player, problem.layout.exit)
    estimatedist = (euclidean+manhattan)/2
    return estimatedist
