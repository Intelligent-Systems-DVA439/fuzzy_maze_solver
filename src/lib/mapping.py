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

# Base Libraries
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
def update_state_value(state_map, path_list, reward_list, alpha = 0.5, gamma = 1):
    # Update states in reverse order
    # Better since formula depends on best NEXT state, so going backwards results in faster convergance than going forwards
    for idx in range(len(path_list) - 1, -1, -1):
        # Unless goal state
        if not path_list[idx].goal:
            # Has neighbors
            if path_list[idx].edges:
                # First neighbor is best successor in the beginning
                best_successor = state_map[path_list[idx].edges[0].end.tostring()].value
                for edge in path_list[idx].edges:
                    # Shortest path is minimization problem
                    if (state_map[edge.end.tostring()].value < best_successor):
                        best_successor = state_map[edge.end.tostring()].value
            # Only goal state should be without neighbors. This state must thus be some dangerous outlier or error, avoid it
            else:
                best_successor = 10000
            # Based on Q-learning monte carlo stochastic model
            path_list[idx].value = path_list[idx].value + alpha * (reward_list[idx] + gamma * best_successor - path_list[idx].value)
        # Goal state has value 0 (minimum)
        else:
            path_list[idx].value = 0

    # Nothing to return
    return None
#==============================================================================

#==============================================================================
# Creates a map using states
def state_mapping(fuzzy_linear, state_map, path_list, reward_list, previous_state):
    # NOTE! np array can not be used as a hash key, hence converting it to string

    # Wait until velocity has a value (aka until the turtlebot velocity message has been received)
    while(shared_variables.velocity == None):
        pass

    # Wait until position has a value (aka until the turtlebot position message has been received)
    while(shared_variables.position == None):
        pass


    # Get current state (global position)
    current_state = np.round(np.concatenate((shared_variables.position.x, shared_variables.position.y), axis=None), 1)
    # First time, use current_state as previous_state
    # After first time, previous state is provided as an argument and is the last loops current value (which is returned at the end)
    if(np.all(previous_state == -1)):
        previous_state = current_state

    # If the new/current state is not in state map, create it and add it
    if current_state.tostring() not in state_map:
        # If outside maze, then it's goal position
        if((shared_variables.position.x > shared_variables.MAZE_BOUNDARY_COORDINATE) |
           (shared_variables.position.x < -shared_variables.MAZE_BOUNDARY_COORDINATE) |
           (shared_variables.position.y > shared_variables.MAZE_BOUNDARY_COORDINATE) |
           (shared_variables.position.y < -shared_variables.MAZE_BOUNDARY_COORDINATE)):
            goal = True
        else:
            goal = False
        state_map[current_state.tostring()] = State(current_state[0], current_state[1], 0, goal)

    # Unless goal state (goal state shouldn't have edges going from it)
    if not state_map[previous_state.tostring()].goal:
        # Add edge from previous state to the new/current state, unless they are the same state
        if not np.array_equal(previous_state, current_state):
            # Add edge only if it does not already exist
            temp_edge = Edge(previous_state, current_state, shared_variables.velocity.angular.z)
            if temp_edge not in state_map[previous_state.tostring()].edges:
                state_map[previous_state.tostring()].add_edge(temp_edge)

    # Append current state to path/traversal list
    path_list.append(state_map[current_state.tostring()])

    # If about to hit wall, set only current states reward to 10 (penalty)
    if(fuzzy_linear <= 0):
        reward_list.append(10)
    else:
        reward_list.append(0)
    # Increment reward for all by 1 each time step (to emphasize finding shortest path)
    for i in range(len(reward_list)):
        reward_list[i] += 1


    # If we are in goal state, update value of all states, flush path and reward list after
    if((shared_variables.position.x > shared_variables.MAZE_BOUNDARY_COORDINATE) |
       (shared_variables.position.x < -shared_variables.MAZE_BOUNDARY_COORDINATE) |
       (shared_variables.position.y > shared_variables.MAZE_BOUNDARY_COORDINATE) |
       (shared_variables.position.y < -shared_variables.MAZE_BOUNDARY_COORDINATE)):
        # Update value of every state
        update_state_value(state_map, path_list, reward_list)

        # Flush path list once goal is reached
        path_list.clear()
        # Flush reward list once goal is reached
        reward_list.clear()


    return current_state
#==============================================================================
