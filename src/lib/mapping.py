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
    def __init__(self, start, end, direction):
        self.start = start
        self.end = end
        self.direction = direction
    def __repr__(self):
        return f"({self.start}, {self.end}, {self.direction})"
#------------------------------------------------------------------------------
# A class for the nodes (states) in the graph
class State:
    def __init__(self, sensor_value, value, goal):
        self.sensor_value = sensor_value
        self.value = value
        self.goal = goal
        self.edges = []
    def __repr__(self):
        return f"Node({self.sensor_value}, {self.value}, {self.goal})"
    def add_edge(self, edge):
        self.edges.append(edge)
#------------------------------------------------------------------------------


#==============================================================================
# Update state values, using an algorihtm inspired by stochastic Q-learning monte carlo
def update_state_value(state_map, path_list, alpha = 0.5, gamma = 0.2):
    for state in reversed(path_list):
        # Unless goal state
        if not state.goal:
            # Has neighbors
            if state.edges:
                # First neighbor is best successor in the beginning
                best_successor = state_map[state.edges[0].end.tostring()].value
                for edge in state.edges:
                    if (state_map[edge.end.tostring()].value < best_successor):
                        best_successor = state_map[edge.end.tostring()].value
            else:
                best_successor = 0
            # Based on Q-learning monte carlo stochastic model
            state.value = state.value + alpha * (1 + gamma * best_successor - state.value)
#==============================================================================

#==============================================================================
# Creates a map using states (sensor values)
# state_map is a hash_map. previous_state and current_state are sensor_value
# goal is a bool for if we are in goal state
def state_mapping(state_map, path_list, previous_state, np_sensor_data):
    # NOTE! np array can not be used as a hash key, hence converting it to string

    # Wait until velocity has a value (aka until the turtlebot velocity message has been received)
    while(shared_variables.velocity == None):
        pass


    # Get current state
    # Split into cones and take mean and then round values to prevent sensor errors from "creating" new states, despite being in the same spot
    # Most stable setup i've managed to find, quite sure sensors have a decent amount of drift
    num_chunks = len(np_sensor_data) // 10
    current_state = np.round(np.array([np.mean(np_sensor_data[i*10:(i+1)*10]) for i in range(num_chunks)]), 0)
    # First time, use current_state as previous_state
    # After first time, previous state is provided as an argument and is the last loops current value (which is returned at the end)
    if(np.all(previous_state == -1)):
        previous_state = current_state


    # If goal is reached, set State to goal state
    if((shared_variables.position.x > 33) | (shared_variables.position.x < -33) | (shared_variables.position.y > 33) | (shared_variables.position.y < -33)):
        goal = True
        print(len(state_map))
        # Flush path list once goal is reached
        path_list = []
    else:
        goal = False

    # If the new/current state is not in state map, create it and add it
    if current_state.tostring() not in state_map:
        state_map[current_state.tostring()] = State(current_state, 0, goal)
        # Add edge from previous state to the new/current state
        if not np.array_equal(previous_state, current_state):
            # Add edge only if does not already exist
            if Edge(previous_state, current_state, shared_variables.velocity.angular.z) not in state_map[previous_state.tostring()].edges:
                state_map[previous_state.tostring()].add_edge(Edge(previous_state, current_state, shared_variables.velocity.angular.z))

    path_list.append(state_map[current_state.tostring()])

    return current_state
#==============================================================================
