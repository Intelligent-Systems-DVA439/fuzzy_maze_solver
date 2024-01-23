#! /usr/bin/env python3

# main.py
#==============================================================================
# Author: Carl Larsson
# Description: AI maze solver
# Date: 16-01-2024

# This software is licensed under the MIT License
# Refer to the LICENSE file for details
#==============================================================================


#------------------------------------------------------------------------------
# Libraries
import math
import threading
import numpy as np
from numpy import inf
import skfuzzy as fuzz
from skfuzzy import control as ctrl

import rclpy
# laser scan message struct
from sensor_msgs.msg import LaserScan
# Movement message struct
from geometry_msgs.msg import Twist
#------------------------------------------------------------------------------


#------------------------------------------------------------------------------
# Global variables

# Global value to receive sensor data (distance/range) from turtlebot
raw_sensor_data = [0 for x in range(360)]
#------------------------------------------------------------------------------
# Fuzzy system


# Fuzzy variables
# Define fuzzy input variables (sensors and their range of possible values)
left_sensor = ctrl.Antecedent(np.arange(0, 3.51, 0.01), 'left_sensor')
front_sensor = ctrl.Antecedent(np.arange(0, 3.51, 0.01), 'front_sensor')
right_sensor = ctrl.Antecedent(np.arange(0, 3.51, 0.01), 'right_sensor')

# Define fuzzy output variable (control signal and its range of possible values)
linear_movement = ctrl.Consequent(np.arange(-0.26, 0.26, 0.01), 'linear')
angular_movement = ctrl.Consequent(np.arange(-1.82, 1.82, 0.01), 'angular')


# Define membership functions, using triangular and trapezoidal memberships
# Sensor readings memberships
left_sensor['close'] = fuzz.trapmf(left_sensor.universe, [-math.inf, 0, 0.4, 1.4]) # "Lower", created using the fist value as being outside of range "to the left"
left_sensor['far'] = fuzz.trapmf(left_sensor.universe, [0.4, 1.4, 3.5, math.inf]) # "Upper", created using the last value outside of range "to the right"

front_sensor['very_close'] = fuzz.trapmf(left_sensor.universe, [-math.inf, 0, 0.2, 0.4]) # "Lower", created using the fist value as being outside of range "to the left"
front_sensor['close'] = fuzz.trimf(left_sensor.universe, [0.2, 0.4, 1])
front_sensor['far'] = fuzz.trapmf(left_sensor.universe, [0.4, 1, 3.5, math.inf]) # "Upper", created using the last value outside of range "to the right"

right_sensor['close'] = fuzz.trapmf(left_sensor.universe, [-math.inf, 0, 0.4, 1.4]) # "Lower", created using the fist value as being outside of range "to the left"
right_sensor['far'] = fuzz.trapmf(left_sensor.universe, [0.4, 1.4, 3.5, math.inf]) # "Upper", created using the last value outside of range "to the right"

# Control output memberships, use tirangular even at the edges since output has limits
# Linear
linear_movement['linear_reverse'] = fuzz.trimf(linear_movement.universe, [-0.26, -0.13, 0.01])
linear_movement['linear_stop'] = fuzz.trimf(linear_movement.universe, [0, 0, 0.8])
linear_movement['linear_forward'] = fuzz.trimf(linear_movement.universe, [0.08, 0.17, 0.26])

# Angular
# z-aix is positive counter clockwise, and negative clockwise (viewed from above)
angular_movement['angular_right'] = fuzz.trimf(angular_movement.universe, [-1.82, -1, 0])
angular_movement['angular_stop'] = fuzz.trimf(angular_movement.universe, [-1, 0, 1]) 
angular_movement['angular_left'] = fuzz.trimf(angular_movement.universe, [0, 1, 1.82])


# Define fuzzy logic rules
# Linear
rule1 = ctrl.Rule(front_sensor['very_close'], linear_movement['linear_reverse'])
rule2 = ctrl.Rule(front_sensor['close'], linear_movement['linear_stop'])
rule3 = ctrl.Rule(front_sensor['far'], linear_movement['linear_forward'])

# Angular
rule4 = ctrl.Rule(left_sensor['far'] & front_sensor['far'] & right_sensor['far'], angular_movement['angular_stop'])
rule5 = ctrl.Rule(left_sensor['close'] & front_sensor['far'] & right_sensor['far'], angular_movement['angular_right'])
rule6 = ctrl.Rule(left_sensor['far'] & (front_sensor['close'] | front_sensor['very_close']) & right_sensor['far'], angular_movement['angular_left']) # Favor left
rule7 = ctrl.Rule(left_sensor['far'] & front_sensor['far'] & right_sensor['close'], angular_movement['angular_left'])
rule8 = ctrl.Rule(left_sensor['far'] & (front_sensor['close'] | front_sensor['very_close']) & right_sensor['close'], angular_movement['angular_left'])
rule9 = ctrl.Rule(left_sensor['close'] & front_sensor['far'] & right_sensor['close'], angular_movement['angular_stop'])
rule10 = ctrl.Rule(left_sensor['close'] & (front_sensor['close'] | front_sensor['very_close']) & right_sensor['far'], angular_movement['angular_right'])
rule11 = ctrl.Rule(left_sensor['close'] & (front_sensor['close'] | front_sensor['very_close']) & right_sensor['close'], angular_movement['angular_left']) # Favor left


# Create fuzzy control system
fuzzy_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11])
fuzzy_system = ctrl.ControlSystemSimulation(fuzzy_ctrl)
#------------------------------------------------------------------------------


#==============================================================================
# Get sensor readings
def get_sensor_readings():
    # Create subscriber/listener node
    node = rclpy.create_node('sensor_subscriber')

    # The function executed each time a message is received
    def listener_callback(msg):
        # Required so the array can be changed
        global raw_sensor_data

        # Store data in a global array
        raw_sensor_data = msg.ranges

    # Subscribe to the topic for the lidar/LaserScan
    # ros2 topic info /scan
    # https://www.youtube.com/watch?v=RFNNsDI2b6c&t=1s
    subscription = node.create_subscription(LaserScan, '/scan', listener_callback, 10)
    subscription  # prevent unused variable warning

    # Spin until work is complete
    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()
#==============================================================================

#==============================================================================
# Maze solver, decide which movement should be taken
def movement_choice():
    # Get sensor values (percept)
    np_sensor_data = np.array(raw_sensor_data)
    np_sensor_data[np_sensor_data == inf] = 3.5

    # Make sure array isn't empty
    if(np_sensor_data.size != 0):
        # Provide sensor values to fuzzy system
        # Lidar values go counter clockwise and start infront of the robot
        # Left value is min value of a 80 degree cone to the left
        fuzzy_system.input['left_sensor'] = np.mean(np_sensor_data[11:90])
        # Front value is min value of a 20 degree cone forward
        fuzzy_system.input['front_sensor'] = np.min(np.concatenate((np_sensor_data[-10:], np_sensor_data[0:10]), axis=None))
        # Right value is min value of a 80 degree cone to the right
        fuzzy_system.input['right_sensor'] = np.mean(np_sensor_data[-90:-11])

        # Fuzzy computation
        fuzzy_system.compute()

        # Fuzzy decision on which movement should be taken
        linear_value = fuzzy_system.output['linear']
        angular_value = fuzzy_system.output['angular']

        return linear_value, angular_value

    return 0, 0
#==============================================================================

#==============================================================================
# Controls turtlebot in gazebo
def robot_control():
    # Create publisher node
    # https://www.youtube.com/watch?v=yEwi1__NJrE
    node = rclpy.create_node('movement_publisher')
    # Publish on command topic
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    # set message to correct struct type
    msg = Twist()

    while(1):
        # Decide which linear and angular movement should be taken
        linear_value, angular_value = movement_choice()
        msg.linear.x = linear_value
        msg.angular.z = angular_value
        # Send message
        publisher.publish(msg)

    # Destroy node explicitly
    node.destroy_node()
    rclpy.shutdown()
#==============================================================================

#==============================================================================
# Main
def main():
    # Initialize rclpy
    rclpy.init()

    # Create thread for taking sensor values
    t1 = threading.Thread(target=get_sensor_readings, name='t1')
    # Create thread for controling robot
    t2 = threading.Thread(target=robot_control, name='t2')

    # Start threads
    t1.start()
    t2.start()

    # Close threads once completed
    t1.join()
    t2.join()
#==============================================================================

#==============================================================================
if __name__ == "__main__":
    main()
#==============================================================================
