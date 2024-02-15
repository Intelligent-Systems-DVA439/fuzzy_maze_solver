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

#------------------------------------------------------------------------------


#------------------------------------------------------------------------------
# A class for edges in the graph
class Edge:
    def __init__(self, start, end, cost):
        self.start = start
        self.end = end
        self.cost = -cost
    def __repr__(self):
        return f"({self.start}, {self.end}, {self.cost:>2})"
#------------------------------------------------------------------------------
# A class for the nodes (states) in the graph
class State:
    def __init__(self, sensor_value, value):
        self.sensor_value = sensor_value
        self.value = value
        self.edges = []
    def __repr__(self):
        return f"Node({self.name}, {self.value:>3}, Edges: {self.edges})"
    def add_edge(self, edge):
        self.edges.append(edge)
#------------------------------------------------------------------------------


#==============================================================================
# Creates a map using states (sensor values)
# state_map is a hash_map. previous_state and current_state are sensor_value
# goal is a bool for if we are in goal state
def state_mapping(state_map, previous_state, current_state, goal_state):

    # If the new/current state is not in state map, add it
    if current_state not in state_map:
        if goal_state == True:
            reward = 0
        else:
            reward = -1

        state_map[current_state] = State(current_state, reward)
        # Add edge from previous state to the new state
        state_map[previous_state].add_edge(Edge(previous_state, current_state, reward))




#==============================================================================
