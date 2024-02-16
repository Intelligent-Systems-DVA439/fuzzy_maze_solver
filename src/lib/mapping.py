# mapping.py
#==============================================================================
# Author: Carl Larsson
# Description: mapping related functions
# Date: 15-02-2024

# This software is licensed under the MIT License
# Refer to the LICENSE file for details
#==============================================================================


#------------------------------------------------------------------------------
# Libraries

# Functional libraries
import numpy as np

# Project libraries
from lib import shared_variables
#------------------------------------------------------------------------------


#------------------------------------------------------------------------------
# A class for edges in the graph
class Edge:
    def __init__(self, start, end, cost):
        self.start = start
        self.end = end
        self.cost = cost
    def __repr__(self):
        return f"({self.start}, {self.end}, {self.cost:>2})"
#------------------------------------------------------------------------------
# A class for the nodes (states) in the graph
class State:
    def __init__(self, sensor_value, value, goal):
        self.sensor_value = sensor_value
        self.value = value
        self.goal = goal
        self.edges = []
    def __repr__(self):
        return f"Node({self.sensor_value})"
    def add_edge(self, edge):
        self.edges.append(edge)
#------------------------------------------------------------------------------


#==============================================================================
# Creates a map using states (sensor values)
# state_map is a hash_map. previous_state and current_state are sensor_value
# goal is a bool for if we are in goal state
def state_mapping(state_map, previous_state, current_state):
    # NOTE! np array can not be used as a hash key, hence converting it to string

    # If goal is reached, set State to goal state
    if((shared_variables.position.x > 33) | (shared_variables.position.x < -33) | (shared_variables.position.y > 33) | (shared_variables.position.y < -33)):
        goal = 1
    else:
        goal = 0

    # If the new/current state is not in state map, add it
    if current_state.tostring() not in state_map:
        state_map[current_state.tostring()] = State(current_state, 0, goal)
        # Add edge from previous state to the new/current state
        if not np.array_equal(previous_state, current_state):
            state_map[previous_state.tostring()].add_edge(Edge(previous_state, current_state, 0))
#==============================================================================
