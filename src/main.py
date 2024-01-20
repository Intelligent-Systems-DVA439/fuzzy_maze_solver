#! /usr/bin/env python3

# main.py
#==============================================================================
# Author: Carl Larsson
# Description: AI maze solver
# To start the gazebo simulation run:
# ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
# Date: 16-01-2024

# This software is licensed under the MIT License
# Refer to the LICENSE file for details
#==============================================================================


#------------------------------------------------------------------------------
# Libraries
import math
import threading
import numpy as np
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

# Global value to receive sensor message from turtlebot
raw_sensor_data = [0 for x in range(360)]
#------------------------------------------------------------------------------
# Fuzzy variables

# Define fuzzy input variables (sensors and their range of possible values)
obstacle_left = ctrl.Antecedent(np.arange(0, 3.51, 0.01), 'obstacle_left')
obstacle_front = ctrl.Antecedent(np.arange(0, 3.51, 0.01), 'obstacle_front')
obstacle_right = ctrl.Antecedent(np.arange(0, 3.51, 0.01), 'obstacle_right')

# Define fuzzy output variable (control signal and its range of possible values)
linear_movement = ctrl.Consequent(np.arange(0, 0.26, 0.01), 'linear')
angular_movement = ctrl.Consequent(np.arange(-1.82, 1.82, 0.01), 'angular')

# Define membership functions, using triangular and trapezoidal memberships
# Sensor readings memberships
obstacle_left['yes'] = fuzz.trapmf(obstacle_left.universe, [-1, 0, 0.5, 1]) # "Lower", created using the fist value as being outside of range "to the left"
obstacle_left['no'] = fuzz.trapmf(obstacle_left.universe, [0.5, 1, 3.5, 4.5]) # "Upper", created using the last value outside of range "to the right"

obstacle_front['yes'] = fuzz.trapmf(obstacle_left.universe, [-1, 0, 0.5, 1]) # "Lower", created using the fist value as being outside of range "to the left"
obstacle_front['no'] = fuzz.trapmf(obstacle_left.universe, [0.5, 1, 3.5, 4.5]) # "Upper", created using the last value outside of range "to the right"

obstacle_right['yes'] = fuzz.trapmf(obstacle_left.universe, [-1, 0, 0.5, 1]) # "Lower", created using the fist value as being outside of range "to the left"
obstacle_right['no'] = fuzz.trapmf(obstacle_left.universe, [0.5, 1, 3.5, 4.5]) # "Upper", created using the last value outside of range "to the right"

# Control output memberships, use tirangular even at the edges since output has limits
# Linear
linear_movement['linear_stop'] = fuzz.trimf(linear_movement.universe, [0, 0.08, 0.13])
linear_movement['linear_forward'] = fuzz.trimf(linear_movement.universe, [0.08, 0.13, 0.26])

# Angular
# z-aix is positive counter clockwise, and negative clockwise
angular_movement['angular_right'] = fuzz.trimf(angular_movement.universe, [-1.82, -1, 0])
angular_movement['angular_stop'] = fuzz.trimf(angular_movement.universe, [-1, 0, 1]) 
angular_movement['angular_left'] = fuzz.trimf(angular_movement.universe, [0, 1, 1.82])

# Define fuzzy logic rules (always favor left)
# Linear
rule1 = ctrl.Rule(obstacle_left['no'] & obstacle_front['no'] & obstacle_right['no'], linear_movement['linear_forward'])
rule2 = ctrl.Rule(obstacle_left['yes'] & obstacle_front['no'] & obstacle_right['no'], linear_movement['linear_forward'])
rule3 = ctrl.Rule(obstacle_left['no'] & obstacle_front['yes'] & obstacle_right['no'], linear_movement['linear_stop'])
rule4 = ctrl.Rule(obstacle_left['no'] & obstacle_front['no'] & obstacle_right['yes'], linear_movement['linear_forward'])
rule5 = ctrl.Rule(obstacle_left['no'] & obstacle_front['yes'] & obstacle_right['yes'], linear_movement['linear_stop'])
rule6 = ctrl.Rule(obstacle_left['yes'] & obstacle_front['no'] & obstacle_right['yes'], linear_movement['linear_forward'])
rule7 = ctrl.Rule(obstacle_left['yes'] & obstacle_front['yes'] & obstacle_right['no'], linear_movement['linear_stop'])
rule8 = ctrl.Rule(obstacle_left['yes'] & obstacle_front['yes'] & obstacle_right['yes'], linear_movement['linear_stop'])
# Angular
rule9 = ctrl.Rule(obstacle_left['no'] & obstacle_front['no'] & obstacle_right['no'], angular_movement['angular_stop'])
rule10 = ctrl.Rule(obstacle_left['yes'] & obstacle_front['no'] & obstacle_right['no'], angular_movement['angular_right'])
rule11 = ctrl.Rule(obstacle_left['no'] & obstacle_front['yes'] & obstacle_right['no'], angular_movement['angular_left']) # Favor left
rule12 = ctrl.Rule(obstacle_left['no'] & obstacle_front['no'] & obstacle_right['yes'], angular_movement['angular_left'])
rule13 = ctrl.Rule(obstacle_left['no'] & obstacle_front['yes'] & obstacle_right['yes'], angular_movement['angular_left'])
rule14 = ctrl.Rule(obstacle_left['yes'] & obstacle_front['no'] & obstacle_right['yes'], angular_movement['angular_stop'])
rule15 = ctrl.Rule(obstacle_left['yes'] & obstacle_front['yes'] & obstacle_right['no'], angular_movement['angular_right'])
rule16 = ctrl.Rule(obstacle_left['yes'] & obstacle_front['yes'] & obstacle_right['yes'], angular_movement['angular_left']) # Favor left


# Create fuzzy control system
fuzzy_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14, rule15, rule16])
fuzzy_system = ctrl.ControlSystemSimulation(fuzzy_ctrl)
#------------------------------------------------------------------------------


#==============================================================================
# Get sensor readings
def get_sensor_readings():
    # Create subscriber/listener node
    node = rclpy.create_node('sensor_subscriber')

    # The function executed each time a message is received
    def listener_callback(msg):
        global raw_sensor_data

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

    if(np_sensor_data.size != 0):
        # Provide sensor values to fuzzy system
        # Lidar values go counter clockwise and start infront of the robot
        # Left value is min value of a 105 degree cone to the left
        fuzzy_system.input['obstacle_left'] = np.min(np_sensor_data[16:90])
        # Front value is min value of a 60 degree cone forward
        fuzzy_system.input['obstacle_front'] = np.min(np.concatenate((np_sensor_data[-15:], np_sensor_data[0:15]), axis=None))
        # Right value is min value of a 105 degree cone to the right
        fuzzy_system.input['obstacle_right'] = np.min(np_sensor_data[-90:-16])

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
