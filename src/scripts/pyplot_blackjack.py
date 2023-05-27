import sys
import random
import gym
import numpy as np
from collections import defaultdict
from plot_utils import plot_blackjack_values, plot_policy

V = {}
with open('blackjack_value_function.txt', 'r') as file:
    for line in file:
        state_first, state_second, value = line.strip().split(' ')
        state = (int(state_first), int(state_second))
        V[state] = float(value)
        
plot_blackjack_values(V)