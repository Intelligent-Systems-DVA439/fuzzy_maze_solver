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
import random

# Functional libraries
import numpy as np

# rclpy libraries
import rclpy
# Movement message struct
from geometry_msgs.msg import Twist

# Project libraries
from lib import shared_variables
from lib.mapping import state_mapping, update_state_value
#------------------------------------------------------------------------------


#------------------------------------------------------------------------------
# Class for range of values, containing min and max value
class Range:
    def __init__(self, min_val, max_val):
        self.min_val = min_val
        self.max_val = max_val
    def __repr__(self):
        return f"({self.min_val}, {self.max_val})"
#------------------------------------------------------------------------------


#==============================================================================
# Fuzzy movement choice
def fuzzy_movement_choice(np_sensor_data, fuzzy_system, sensor, linear, angular):
    # Provide normalized sensor values to fuzzy system
    # Lidar values go counter clockwise and start infront of the robot
    # Left value is mean value of a 70 degree cone to the left
    fuzzy_system.input['left_sensor'] = (np.mean(np_sensor_data[10:80]) - sensor.min_val)/(sensor.max_val - sensor.min_val)
    # Front value is min value of a 40 degree cone forward
    fuzzy_system.input['front_sensor'] = (np.min(np.concatenate((np_sensor_data[-20:], np_sensor_data[0:20]), axis=None)) - sensor.min_val)/(sensor.max_val - sensor.min_val)
    # Right value is mean value of a 70 degree cone to the right
    fuzzy_system.input['right_sensor'] = (np.mean(np_sensor_data[-80:-10]) - sensor.min_val)/(sensor.max_val - sensor.min_val)

    # Fuzzy computation
    fuzzy_system.compute()

    # Fuzzy decision on which movement should be taken
    # Also "unnormalize" the data
    linear_value = (fuzzy_system.output['linear'] - (-1)) * (linear.max_val - linear.min_val) / (1 - (-1)) + linear.min_val
    angular_value = (fuzzy_system.output['angular'] - (-1)) * (angular.max_val - angular.min_val) / (1 - (-1)) + angular.min_val

    return linear_value, angular_value
#==============================================================================

#==============================================================================
# Decides which direction to go according to global pathing
def pathing_direction(fuzzy_angular, state_map, current_state):
    # NOTE! np array can not be used as a hash key, hence converting it to string

    # Check if current state has any neighbors (is not empty)
    if state_map[current_state.tostring()].edges:
        # To begin with, the best state is just the first neighbor
        best_edge = state_map[current_state.tostring()].edges[0]
        for edge in state_map[current_state.tostring()].edges:
            # Shortest path is minimization problem, thus lower value is better
            if(state_map[edge.end.tostring()].value < state_map[best_edge.end.tostring()].value):
                best_edge = edge

        # Find which direction should be taken to get to best state next
        # Just changes the direction by doing sign change
        # Both agree, don't change direction
        if(np.sign(best_edge.direction) == np.sign(fuzzy_angular)):
            direction = 1
        # Change direction to oposite
        else:
            direction = -1

        # Random exploration, chance is reduced the more the maze is explored
        # 1 run roughly increases the length of state_map by 1300
        random_value = random.randint(0, 100000)
        if (len(state_map) < random_value):
            direction = -direction

    # No neighbors, then global pathing can't help and it gives no input
    else:
        direction = 1

    return direction
#==============================================================================

#==============================================================================
# Global pathing and mapping
def global_pathing(np_sensor_data, fuzzy_linear, fuzzy_angular, state_map, path_list, previous_state):
    # Create current state and do mapping of the maze
    current_state = state_mapping(np_sensor_data, state_map, path_list, previous_state)

    # Update value of every state
    update_state_value(fuzzy_linear, state_map, path_list)

    # Find which direction should be taken according to global pathing
    direction = pathing_direction(fuzzy_angular, state_map, current_state)
    
    return current_state, direction
#==============================================================================

#==============================================================================
# Decide which movement should be taken
def movement_choice(fuzzy_system, sensor, linear, angular, state_map, path_list, previous_state):
    # Get sensor values (percept)
    np_sensor_data = np.array(shared_variables.raw_sensor_data)

    # Stuck until sensor has been initialized and sensor values have been received
    while(np.all(np_sensor_data == -1)):
        np_sensor_data = np.array(shared_variables.raw_sensor_data)

    # Set all inf values to max value since average is calculated later
    np_sensor_data[np_sensor_data == math.inf] = 3.5


    # Fuzzy
    fuzzy_linear, fuzzy_angular = fuzzy_movement_choice(np_sensor_data, fuzzy_system, sensor, linear, angular)

    # Global path algorithm and mapping
    current_state, global_pathing_direction = global_pathing(np_sensor_data, fuzzy_linear, fuzzy_angular, state_map, path_list, previous_state)

    # Final movement choice
    # Linear velocity
    linear_value = fuzzy_linear
    # Turning is based on fuzzy, with input (sign change) from global path on where to turn
    angular_value = global_pathing_direction*fuzzy_angular

    return linear_value, angular_value, current_state
#==============================================================================

#==============================================================================
# Controls turtlebot in gazebo
def robot_control(node_array, fuzzy_system, sensor, linear, angular, state_map):
    # Create publisher node
    node = rclpy.create_node('movement_publisher')
    # Publish on command topic
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    # Add node to node array for shutdown
    node_array.append(node)

    # set message to correct struct type
    msg = Twist()

    # Variables for global pathing and mapping
    path_list = []
    current_state = np.full((362, 1), -1)

    while(shared_variables.shutdown_flag != True):
        # Decide which linear and angular movement should be taken
        linear_value, angular_value, current_state = movement_choice(fuzzy_system, sensor, linear, angular, state_map, path_list, current_state)
        msg.linear.x = linear_value
        msg.angular.z = angular_value

        # Movement is set to 0 if goal is reached to not influence beginning after reset
        if((shared_variables.position.x > shared_variables.maze_boundary_coordinate) |
           (shared_variables.position.x < -shared_variables.maze_boundary_coordinate) |
           (shared_variables.position.y > shared_variables.maze_boundary_coordinate) |
           (shared_variables.position.y < -shared_variables.maze_boundary_coordinate)):
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        # Send message
        publisher.publish(msg)

    # Only returns on shutdown
    return None
#==============================================================================
