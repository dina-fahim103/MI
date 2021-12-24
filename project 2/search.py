from typing import Tuple
from game import HeuristicFunction, Game, S, A
from helpers.utils import NotImplemented
import math
from operator import itemgetter

# All search functions take a problem, a state, a heuristic function and the maximum search depth.
# If the maximum search depth is -1, then there should be no depth cutoff (The expansion should not stop before reaching a terminal state)

# All the search functions should return the expected tree value and the best action to take based on the search results

# This is a simple search function that looks 1-step ahead and returns the action that lead to highest heuristic value.
# This algorithm is bad if the heuristic function is weak. That is why we use minimax search to look ahead for many steps.


def greedy(game: Game[S, A], state: S, heuristic: HeuristicFunction, max_depth: int = -1) -> Tuple[float, A]:
    agent = game.get_turn(state)

    terminal, values = game.is_terminal(state)
    if terminal:
        return values[agent], None

    actions_states = [(action, game.get_successor(state, action))
                      for action in game.get_actions(state)]
    value, _, action = max((heuristic(game, state, agent), -index, action)
                           for index, (action, state) in enumerate(actions_states))
    return value, action

# Apply Minimax search and return the tree value and the best action
# This is a minimax search function that uses recursion to look as many steps ahead and returns the action that lead to max best move value.


def minimax_search(game: Game[S, A], state: S, heuristic: HeuristicFunction, max_depth: int = -1) -> Tuple[float, A]:
    terminal, values = game.is_terminal(state)
    if terminal:
        return values[0], None

    if max_depth == 0:  # no steps ahead
        return heuristic(game, state, 0), None
    best_action = None  # initializing best action
    actions_states = [(action, game.get_successor(state, action))
                      for action in game.get_actions(state)]

    if game.get_turn(state) == 0:  # player turn
        expected_value = -math.inf  # initialized with negative infinity for max
        for action, successor in actions_states:
            max_value = minimax_search(
                game, successor, heuristic, max_depth-1)[0]  # recursion one step at a time
            if max_value > expected_value:
                expected_value = max_value
                best_action = action

    else:  # monster turn
        expected_value = math.inf  # initialized with infinity for min
        for action, successor in actions_states:
            min_value = minimax_search(
                game, successor, heuristic, max_depth-1)[0]  # recursion one step at a time
            if min_value < expected_value:
                expected_value = min_value
                best_action = action
    return expected_value, best_action  # return expected value and best action


def minimax(game: Game[S, A], state: S, heuristic: HeuristicFunction, max_depth: int = -1) -> Tuple[float, A]:
    # call minimax search
    return minimax_search(game, state, heuristic, max_depth)

# Apply Alpha Beta pruning and return the tree value and the best action
# This is an alphabeta search function that uses recursion to look as many steps ahead and returns the action that lead to max best move value with pruning unneeded moves.


def alphabeta_search(game: Game[S, A], state: S, heuristic: HeuristicFunction, max_depth: int, alpha: float, beta: float) -> Tuple[float, A]:

    terminal, values = game.is_terminal(state)
    if terminal:
        return values[0], None

    if max_depth == 0:  # no steps ahead
        return heuristic(game, state, 0), None
    best_action = None  # initializing best action
    actions_states = [(action, game.get_successor(state, action))
                      for action in game.get_actions(state)]

    if game.get_turn(state) == 0:  # player turn

        expected_value = -math.inf  # initialized with negative infinity for max
        for action, successor in actions_states:
            max_value = alphabeta_search(game, successor, heuristic,
                                         max_depth-1, alpha, beta)[0]  # recursion one step at a time
            if max_value > expected_value:
                expected_value = max_value
                best_action = action
            alpha = max(alpha, expected_value)  # updating alpha value
            if beta <= alpha:  # pruning decision
                break  # pruning

    else:  # monster turn
        expected_value = math.inf  # initialized with infinity for min
        for action, successor in actions_states:
            min_value = alphabeta_search(game, successor, heuristic,
                                         max_depth-1, alpha, beta)[0]  # recursion one step at a time
            if min_value < expected_value:
                expected_value = min_value
                best_action = action
            beta = min(beta, expected_value)  # updating beta value
            if beta <= alpha:  # pruning decision
                break  # pruning
    return expected_value, best_action  # return expected value and best action


def alphabeta(game: Game[S, A], state: S, heuristic: HeuristicFunction, max_depth: int = -1) -> Tuple[float, A]:
    # call alphabeta search
    return alphabeta_search(game, state, heuristic, max_depth, -math.inf, math.inf)

# Apply Alpha Beta pruning with move ordering and return the tree value and the best action
# This is an alphabeta search function that uses recursion and move ordering to look as many steps ahead and returns the action that lead to max best move value with pruning unneeded moves.


def alphabeta_with_move_ordering_search(game: Game[S, A], state: S, heuristic: HeuristicFunction, max_depth: int, alpha: float, beta: float) -> Tuple[float, A]:

    terminal, values = game.is_terminal(state)
    if terminal:
        return values[0], None

    if max_depth == 0:  # no steps ahead
        return heuristic(game, state, 0), None
    best_action = None  # initializing best action
    heuristics_actions_states = [(heuristic(game, game.get_successor(state, action), 0), action, game.get_successor(state, action))
                                 for action in game.get_actions(state)]

    # ascending sorting for min
    actions_states_mini = sorted(heuristics_actions_states, key=itemgetter(0))

    actions_states_max = sorted(
        heuristics_actions_states, key=itemgetter(0), reverse=True)  # descending sorting for max

    if game.get_turn(state) == 0:  # player turn

        expected_value = -math.inf  # initialized with negative infinity for max
        for heuristics, action, successor in actions_states_max:
            max_value = alphabeta_with_move_ordering_search(game, successor, heuristic,
                                                            max_depth-1, alpha, beta)[0]  # recursion one step at a time
            if max_value > expected_value:
                expected_value = max_value
                best_action = action
            alpha = max(alpha, expected_value)  # updating alpha value
            if beta <= alpha:  # pruning decision
                break  # pruning

    else:  # monster turn
        expected_value = math.inf  # initialized with infinity for min
        for heuristics, action, successor in actions_states_mini:
            min_value = alphabeta_with_move_ordering_search(game, successor, heuristic,
                                                            max_depth-1, alpha, beta)[0]  # recursion one step at a time
            if min_value < expected_value:
                expected_value = min_value
                best_action = action
            beta = min(beta, expected_value)  # updating beta value
            if beta <= alpha:  # pruning decision
                break  # pruning
    return expected_value, best_action  # return expected value and best action


def alphabeta_with_move_ordering(game: Game[S, A], state: S, heuristic: HeuristicFunction, max_depth: int = -1) -> Tuple[float, A]:
    # call alphabeta search with move ordering
    return alphabeta_with_move_ordering_search(game, state, heuristic, max_depth, -math.inf, math.inf)

# Apply Expectimax search and return the tree value and the best action
# This is an expectimax search function that uses recursion and probability to look as many steps ahead and returns the action that lead to max best move value.


def expectimax_search(game: Game[S, A], state: S, heuristic: HeuristicFunction, max_depth: int = -1) -> Tuple[float, A]:
    terminal, values = game.is_terminal(state)
    if terminal:
        return values[0], None

    if max_depth == 0:  # no steps ahead
        return heuristic(game, state, 0), None
    best_action = None  # initializing best action
    actions_states = [(action, game.get_successor(state, action))
                      for action in game.get_actions(state)]
    if game.get_turn(state) == 0:  # player turn

        expected_value = -math.inf  # initialized with negative infinity for max
        for action, successor in actions_states:
            max_value = expectimax_search(
                game, successor, heuristic, max_depth-1)[0]  # recursion one step at a time
            if max_value > expected_value:
                expected_value = max_value
                best_action = action

    else:  # monster turn
        expected_value = math.inf  # initialized with infinity for avg
        avg_value = 0.0
        sum_values = 0.0
        no_values = 0
        for action, successor in actions_states:
            no_values += 1
            sum_values += expectimax_search(
                game, successor, heuristic, max_depth-1)[0]  # recursion one step at a time
        avg_value = sum_values/no_values  # calculating the average
        if avg_value < expected_value:
            expected_value = avg_value
            best_action = action
    return expected_value, best_action  # return expected value and best action


def expectimax(game: Game[S, A], state: S, heuristic: HeuristicFunction, max_depth: int = -1) -> Tuple[float, A]:
    # call expectimax search
    return expectimax_search(game, state, heuristic, max_depth)
