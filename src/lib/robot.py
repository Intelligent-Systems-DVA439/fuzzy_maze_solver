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
#------------------------------------------------------------------------------


#==============================================================================
# Maze solver, decide which movement should be taken
def movement_choice(fuzzy_system, min_sensor_value, max_sensor_value, min_linear, max_linear, min_angular, max_angular):
    # Get sensor values (percept)
    np_sensor_data = np.array(shared_variables.raw_sensor_data)

    # Stuck until sensor has been initialized and sensor values have been received
    while(np.all(np_sensor_data == -1)):
        print("sensor_value never received")
        np_sensor_data = np.array(shared_variables.raw_sensor_data)

    # Set all inf values to max value since average is calculated later
    np_sensor_data[np_sensor_data == math.inf] = 3.5

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

    while(shared_variables.shutdown_flag != True):
        # Decide which linear and angular movement should be taken
        linear_value, angular_value = movement_choice(fuzzy_system, min_sensor_value, max_sensor_value, min_linear, max_linear, min_angular, max_angular)
        msg.linear.x = linear_value
        msg.angular.z = angular_value
        # Send message
        publisher.publish(msg)
#==============================================================================
