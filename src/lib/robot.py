# robot.py
#==============================================================================
# Author: Carl Larsson
# Description: functions related to robot control
# Date: 15-02-2024

# This software is licensed under the MIT License
# Refer to the LICENSE file for details
#==============================================================================


#------------------------------------------------------------------------------
# Libraries

# Base libraries
import math
import random
import time

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
def fuzzy_movement_choice(np_sensor_data, fuzzy_system):
    # For normalization and unnormalization
    sensor = Range(0, 3.51)
    linear = Range(-0.26, 0.26)
    angular = Range(-1.82, 1.82)

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
# Exploration function deciding if random exploration action should be taken
def exploration_function(state_map, current_state, direction, magnitude):

    # Inverse logarithmic growth probability based on number of edges/neighbors
    def neighbor_probability(x, upper_limit):
        # If more than 8 edges, 0% chance
        if x >= upper_limit:
            return 0.01
        # Log(x) when x<1 is over 1 (which would mean more than 100%)
        elif x <= 1:
            return 0.7
        # In between chance is based on inverse logarithmic growth
        else:
            # Calculate probability using inverse logarithmic growth function
            probability = 1 - math.log(x) / math.log(upper_limit)  # Logarithmic growth from 100% to 0%
            return probability


    # Calculate chance based on number or edges of current state, 0 edges 100% chance and then dropping off to 0% in a inverse logarithmic growth curve
    neighbor_chance = neighbor_probability(len(state_map[current_state.tostring()].edges), 6)

    # Chance to take random direction
    if(random.random() < neighbor_chance):
        random_direction = -direction
    else:
        random_direction = direction

    # Chance to make the magnitude of the direction random
    if(random.random() < neighbor_chance):
        random_magnitude = random.random()
    else:
        random_magnitude = magnitude
        
    return random_direction, random_magnitude
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
        # Both agree, don't change direction
        if(np.sign(best_edge.direction) == np.sign(fuzzy_angular)):
            direction = 1
        # Change direction to oposite
        else:
            direction = -1

        # Decide on angular magnitude
        magnitude = abs(fuzzy_angular)/abs(best_edge.direction)

        # Random exploration, chance is based on number of edges and number of new states/edges dicovered last epoch
        direction, magnitude = exploration_function(state_map, current_state, direction, magnitude)

    # No neighbors, then global pathing can't help and it gives no input
    else:
        direction = 1
        magnitude = 1

    return direction*magnitude
#==============================================================================

#==============================================================================
# Global pathing and mapping
def global_pathing(fuzzy_linear, fuzzy_angular, state_map, path_list, reward_list, previous_state):
    # Create current state and do mapping of the maze
    current_state = state_mapping(fuzzy_linear, state_map, path_list, reward_list, previous_state)

    # Find which direction should be taken according to global pathing
    direction = pathing_direction(fuzzy_angular, state_map, current_state)
    
    return current_state, direction
#==============================================================================

#==============================================================================
# Decide which movement should be taken
def movement_choice(fuzzy_system, state_map, path_list, reward_list, previous_state, start_time):
    # Get sensor values (percept)
    np_sensor_data = np.array(shared_variables.raw_sensor_data)

    # Stuck until sensor has been initialized and sensor values have been received
    while(np.all(np_sensor_data == -1)):
        np_sensor_data = np.array(shared_variables.raw_sensor_data)

    # Set all inf values to max value since average is calculated later
    np_sensor_data[np_sensor_data == math.inf] = 3.5


    # Fuzzy
    fuzzy_linear, fuzzy_angular = fuzzy_movement_choice(np_sensor_data, fuzzy_system)

    # Global path algorithm and mapping
    previous_state, global_pathing_direction = global_pathing(fuzzy_linear, fuzzy_angular, state_map, path_list, reward_list, previous_state)

    # Final movement choice
    # Linear velocity
    linear_value = fuzzy_linear
    # Unless its taken 30min
    if((time.time() - start_time) < 1800):
        # Turning is based on fuzzy, with input from global path on where to turn
        angular_value = global_pathing_direction*fuzzy_angular
    else:
        # Turning only based on fuzzy
        angular_value = fuzzy_angular

    return linear_value, angular_value, previous_state
#==============================================================================

#==============================================================================
# Controls turtlebot in gazebo
def robot_control(node_array, fuzzy_system, state_map):
    # Create publisher node
    node = rclpy.create_node('movement_publisher')
    # Publish on command topic (buffer size 1 since old commands should not be executed)
    publisher = node.create_publisher(Twist, '/cmd_vel', 1)
    # Add node to node array for shutdown
    node_array.append(node)

    # set message to correct struct type
    msg = Twist()

    # Variables for global pathing and mapping
    path_list = []
    reward_list = []
    previous_state = np.full((362, 1), -1)

    # Used for overide incase solving takes to long 
    start_time = time.time()

    # Wait until position has a value (aka until the turtlebot position message has been received)
    while(shared_variables.position == None):
        pass

    while(shared_variables.shutdown_flag != True):
        # Decide which linear and angular movement should be taken
        linear_value, angular_value, previous_state = movement_choice(fuzzy_system, state_map, path_list, reward_list, previous_state, start_time)
        msg.linear.x = linear_value
        msg.angular.z = angular_value

        # Movement is set to 0 if goal is reached to not influence beginning after reset
        if((shared_variables.position.x > shared_variables.MAZE_BOUNDARY_COORDINATE) |
           (shared_variables.position.x < -shared_variables.MAZE_BOUNDARY_COORDINATE) |
           (shared_variables.position.y > shared_variables.MAZE_BOUNDARY_COORDINATE) |
           (shared_variables.position.y < -shared_variables.MAZE_BOUNDARY_COORDINATE)):
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            # Reset time
            start_time = time.time()

        # Request reseting everything if 45 min has passed
        if((time.time() - start_time) > 2700):
            shared_variables.reset_request = True
            start_time = time.time()

        # Send message
        publisher.publish(msg)

    # Only returns on shutdown
    return None
#==============================================================================
