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
    def __init__(self, x, y, value, goal):
        self.x = x
        self.y = y
        self.value = value
        self.goal = goal
        self.edges = []
    def __repr__(self):
        return f"Node({self.value}, {self.goal})"
    def add_edge(self, edge):
        self.edges.append(edge)
#------------------------------------------------------------------------------


#==============================================================================
# Update state values, using an algorihtm inspired by stochastic Q-learning monte carlo
def update_state_value(fuzzy_linear, state_map, path_list, alpha = 0.5, reward = 1, gamma = 1):
    # Update states in reverse order
    # Better since formula depends on best NEXT state, so going backwards results in faster convergance than going forwards
    for state in reversed(path_list):
        # Unless goal state
        if not state.goal:
            # Has neighbors
            if state.edges:
                # First neighbor is best successor in the beginning
                best_successor = state_map[state.edges[0].end.tostring()].value
                for edge in state.edges:
                    # Shortest path is minimization problem
                    if (state_map[edge.end.tostring()].value < best_successor):
                        best_successor = state_map[edge.end.tostring()].value
            else:
                best_successor = 0
            # If about to hit wall, set only current states reward to 500 times more (substantial penalty)
            if((fuzzy_linear < 0) & (state == path_list[-1])):
                # Based on Q-learning monte carlo stochastic model
                state.value = state.value + alpha * (500*reward + gamma * best_successor - state.value)
            else:
                # Based on Q-learning monte carlo stochastic model
                state.value = state.value + alpha * (reward + gamma * best_successor - state.value)
        # Goal state has value 0 (minimum)
        else:
            state.value = 0

    # Nothing to return
    return None
#==============================================================================

#==============================================================================
# Creates a map using states (sensor values and global position)
# state_map is a hash_map. previous_state and current_state are np arrays
# goal is a bool for if we are in goal state
def state_mapping(np_sensor_data, state_map, path_list, previous_state):
    # NOTE! np array can not be used as a hash key, hence converting it to string

    # Wait until velocity has a value (aka until the turtlebot velocity message has been received)
    while(shared_variables.velocity == None):
        pass


    # Get current state
    # All sensor values, rounded to 2 decimals, and global position
    current_state = np.concatenate((shared_variables.position.x, shared_variables.position.y, np.round(np_sensor_data, 2)), axis=None)
    # First time, use current_state as previous_state
    # After first time, previous state is provided as an argument and is the last loops current value (which is returned at the end)
    if(np.all(previous_state == -1)):
        previous_state = current_state


    # If goal is reached, set State to goal state and flush path list
    if((shared_variables.position.x > shared_variables.maze_boundary_coordinate) |
       (shared_variables.position.x < -shared_variables.maze_boundary_coordinate) |
       (shared_variables.position.y > shared_variables.maze_boundary_coordinate) |
       (shared_variables.position.y < -shared_variables.maze_boundary_coordinate)):
        # Flush path list once goal is reached
        path_list = []
        goal = True
    else:
        goal = False

    # If the new/current state is not in state map, create it and add it
    if current_state.tostring() not in state_map:
        state_map[current_state.tostring()] = State(current_state[0], current_state[1], 0, goal)
        # Add edge from previous state to the new/current state
        if not np.array_equal(previous_state, current_state):
            # Add edge only if it does not already exist
            if Edge(previous_state, current_state, shared_variables.velocity.angular.z) not in state_map[previous_state.tostring()].edges:
                state_map[previous_state.tostring()].add_edge(Edge(previous_state, current_state, shared_variables.velocity.angular.z))

    # Append current state to path/traversal list
    path_list.append(state_map[current_state.tostring()])

    return current_state
#==============================================================================
