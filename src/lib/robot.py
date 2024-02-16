# robot.py
#==============================================================================
# Author: Carl Larsson
# Description: functions related to the robot
# Date: 15-02-2024

# This software is licensed under the MIT License
# Refer to the LICENSE file for details
#==============================================================================


#------------------------------------------------------------------------------
# Libraries

# Base libraries
import math

# Functional libraries
import numpy as np

# rclpy libraries
import rclpy
# Movement message struct
from geometry_msgs.msg import Twist

# Project libraries
from lib import shared_variables
from lib.mapping import state_mapping
#------------------------------------------------------------------------------


#==============================================================================
# Fuzzy movement choice
def fuzzy_movement_choice(fuzzy_system, min_sensor_value, max_sensor_value, min_linear, max_linear, min_angular, max_angular, np_sensor_data):
    # Provide normalized sensor values to fuzzy system
    # Lidar values go counter clockwise and start infront of the robot
    # Left value is mean value of a 70 degree cone to the left
    fuzzy_system.input['left_sensor'] = (np.mean(np_sensor_data[10:80]) - min_sensor_value)/(max_sensor_value - min_sensor_value)
    # Front value is min value of a 40 degree cone forward
    fuzzy_system.input['front_sensor'] = (np.min(np.concatenate((np_sensor_data[-20:], np_sensor_data[0:20]), axis=None)) - min_sensor_value)/(max_sensor_value - min_sensor_value)
    # Right value is mean value of a 70 degree cone to the right
    fuzzy_system.input['right_sensor'] = (np.mean(np_sensor_data[-80:-10]) - min_sensor_value)/(max_sensor_value - min_sensor_value)
    #print("\nL: ",np.mean(np_sensor_data[10:80]),", F: ",np.min(np.concatenate((np_sensor_data[-20:], np_sensor_data[0:20]), axis=None)),", R: ",np.mean(np_sensor_data[-80:-10]),"\n")

    # Fuzzy computation
    fuzzy_system.compute()

    # Fuzzy decision on which movement should be taken
    # Also "unnormalize" the data
    linear_value = (fuzzy_system.output['linear'] - (-1)) * (max_linear - min_linear) / (1 - (-1)) + min_linear
    angular_value = (fuzzy_system.output['angular'] - (-1)) * (max_angular - min_angular) / (1 - (-1)) + min_angular

    return linear_value, angular_value
#==============================================================================

#==============================================================================
# Global pathing and mapping
def global_pathing(np_sensor_data, state_map, path_list, previous_state):
    # Split into cones and take mean and then round values to prevent sensor errors from "creating" new states, despite being in the same spot
    # Most stable setup i've managed to find, quite sure sensors have a decent amount of drift
    num_chunks = len(np_sensor_data) // 10
    current_state = np.round(np.array([np.mean(np_sensor_data[i*10:(i+1)*10]) for i in range(num_chunks)]), 0)
    # First time, use current_state as previous_state
    # After first time, previous state is provided as an argument and is the last loops current value (which is returned at the end)
    if(np.all(previous_state == -1)):
        previous_state = current_state

    # Do mapping of the maze
    state_mapping(state_map, path_list, previous_state, current_state)

    # Update value of every state
    for state in path_list:
        # Unless goal state
        if state.goal != 1:
            # Increment value, since it's a minimization problem, shortest path
            state.value = state.value + 1

    # Find which direction should be taken according to global pathing
    # Check if current state has any neighbors (is not empty)
    if state_map[current_state.tostring()].edges:
        # To begin with, the best state is just the first neighbor
        best = state_map[current_state.tostring()].edges[0]
        for edge in state_map[current_state.tostring()].edges:
            if(state_map[edge.end.tostring()].value < state_map[best.end.tostring()].value):
                best = edge
        # Find which direction should be taken to get to next state
        direction = best.direction
    # No neighbors, then global pathing can't help and it gives no input
    else:
        direction = 0
    
    return current_state, direction
#==============================================================================

#==============================================================================
# Maze solver, decide which movement should be taken
def movement_choice(fuzzy_system, min_sensor_value, max_sensor_value, min_linear, max_linear, min_angular, max_angular, state_map, path_list, previous_state):
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Sensor values

    # Get sensor values (percept)
    np_sensor_data = np.array(shared_variables.raw_sensor_data)

    # Stuck until sensor has been initialized and sensor values have been received
    while(np.all(np_sensor_data == -1)):
        print("sensor_value never received")
        np_sensor_data = np.array(shared_variables.raw_sensor_data)

    # Set all inf values to max value since average is calculated later
    np_sensor_data[np_sensor_data == math.inf] = 3.5
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Fuzzy

    fuzzy_linear, fuzzy_angular = fuzzy_movement_choice(fuzzy_system, min_sensor_value, max_sensor_value, min_linear, max_linear, min_angular, max_angular, np_sensor_data)
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Global path algorithm and mapping

    current_state, global_pathing_direction = global_pathing(np_sensor_data, state_map, path_list, previous_state)
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Final movement choice

    # Linear velocity
    linear_value = fuzzy_linear
    # Where to turn is based on weighted sum of fuzzy and global pathing
    angular_value = 1*fuzzy_angular + 0.2*global_pathing_direction

    return linear_value, angular_value, current_state
#==============================================================================

#==============================================================================
# Controls turtlebot in gazebo
def robot_control(node_array, fuzzy_system, min_sensor_value, max_sensor_value, min_linear, max_linear, min_angular, max_angular):
    # Create publisher node
    # https://www.youtube.com/watch?v=yEwi1__NJrE
    node = rclpy.create_node('movement_publisher')
    # Publish on command topic
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    # Add node to node array for shutdown
    node_array.append(node)

    # set message to correct struct type
    msg = Twist()

    # Variables for global pathing and mapping
    state_map = {}
    path_list = []
    current_state = np.full((360, 1), -1)

    while(shared_variables.shutdown_flag != True):
        # Decide which linear and angular movement should be taken
        linear_value, angular_value, current_state = movement_choice(fuzzy_system, min_sensor_value, max_sensor_value, min_linear, max_linear, min_angular, max_angular, state_map, path_list, current_state)
        msg.linear.x = linear_value
        msg.angular.z = angular_value
        # Send message
        publisher.publish(msg)
#==============================================================================
