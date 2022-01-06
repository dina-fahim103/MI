from typing import Dict
from agents import Agent
from environment import Environment
from mdp import MarkovDecisionProcess, S, A
import json
from helpers.utils import NotImplemented
from collections import defaultdict

# This is a class for a generic Value Iteration agent


class ValueIterationAgent(Agent[S, A]):
    mdp: MarkovDecisionProcess[S, A]  # The MDP used by this agent for training
    utilities: Dict[str, float]  # The computed utilities
    # The key is the string representation of the state and the value is the utility
    discount_factor: float  # The discount factor (gamma)

    def __init__(self, mdp: MarkovDecisionProcess[S, A], discount_factor: float = 0.99) -> None:
        super().__init__()
        self.mdp = mdp
        # We initialize all the utilities to be 0
        self.utilities = {str(state): 0 for state in self.mdp.get_states()}
        self.discount_factor = discount_factor

    # Given a state, compute its utility using the bellman equation
    # if the state is terminal, return 0
    def compute_bellman(self, state: S) -> float:
        # TODO: Complete this function
        utility = -1000.0
        if (self.mdp.is_terminal(state)):
            return 0.0

        for action in self.mdp.get_actions(state):
            utility0 = 0.0
            next_states = self.mdp.get_successor(state, action)
            for next_state in next_states:
                reward = self.mdp.get_reward(state, action, next_state)
                utility0 += (reward+self.discount_factor *
                             self.utilities[str(next_state)])*next_states[next_state]
            utility = max(utility0, utility)
        return utility  # computing according to given ballman eqn
    # This function applies value iteration starting from the current utilities stored in the agent and stores the new utilities in the agent
    # NOTE: this function does incremental update and does not clear the utilities to 0 before running
    # In other words, calling train(M) followed by train(N) is equivalent to just calling train(N+M)

    def train(self, iterations: int = 1):
        # TODO: Complete this function to apply value iteration for the given number of iterations
        for i in range(iterations):
            utilities_dict = {}
            for state in self.mdp.get_states():
                utilities_dict[str(state)] = self.compute_bellman(state)
            self.utilities = utilities_dict  # appending new utilities
# Given an environment and a state, return the best action as guided by the learned utilities and the MDP
# If the state is terminal, return None

    def act(self, env: Environment[S, A], state: S) -> A:
        # TODO: Complete this function
        # if more than one action has the maximum expected utility, return the one that appears first in the "actions" list
        utility = -1000.0
        myaction = None
        if (self.mdp.is_terminal(state)):
            return None
        for action in self.mdp.get_actions(state):
            utility0 = 0.0
            next_states = self.mdp.get_successor(state, action)
            for next_state in next_states:
                reward = self.mdp.get_reward(state, action, next_state)
                utility0 += (reward+self.discount_factor *
                             self.utilities[str(next_state)])*next_states[next_state]
            if utility0 > utility:
                utility = utility0
                myaction = action
        return myaction  # returning best action of best utility

    # Save the utilities to a json file

    def save(self, file_path: str):
        with open(file_path, 'w') as f:
            json.dump(self.utilities, f, indent=2, sort_keys=True)

    # loads the utilities from a json file
    def load(self, file_path: str):
        with open(file_path, 'r') as f:
            self.utilities = json.load(f)
