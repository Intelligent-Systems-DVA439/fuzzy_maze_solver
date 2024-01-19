#! /usr/bin/env python
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
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
#------------------------------------------------------------------------------


#------------------------------------------------------------------------------
# Global variables
raw_sensor_data = None
#------------------------------------------------------------------------------
# Fuzzy variables

# Define fuzzy input variables (sensors and their range of possible values)
obstacle_left = ctrl.Antecedent(np.arange(0, 2, 1), 'obstacle_left')
obstacle_front = ctrl.Antecedent(np.arange(0, 2, 1), 'obstacle_front')
obstacle_right = ctrl.Antecedent(np.arange(0, 2, 1), 'obstacle_right')

# Define fuzzy output variable (control signal and its range of possible values)
movement = ctrl.Consequent(np.arange(-1, 2, 1), 'movement')

# Define membership functions, using triangular and trapezoidal memberships
# Sensor readings memberships
obstacle_left['no'] = fuzz.trapmf(obstacle_left.universe, [0.5, 0.7, 1, 2]) # "Upper", created using the last value outside of range "to the right"
obstacle_left['yes'] = fuzz.trapmf(obstacle_left.universe, [-1, 0, 0,3, 0,5]) # "Lower", created using the fist value as being outside of range "to the left"

obstacle_front['no'] = fuzz.trapmf(obstacle_left.universe, [0.5, 0.7, 1, 2]) # "Upper", created using the last value outside of range "to the right"
obstacle_front['yes'] = fuzz.trapmf(obstacle_left.universe, [-1, 0, 0,3, 0,5]) # "Lower", created using the fist value as being outside of range "to the left"

obstacle_right['no'] = fuzz.trapmf(obstacle_left.universe, [0.5, 0.7, 1, 2]) # "Upper", created using the last value outside of range "to the right"
obstacle_right['yes'] = fuzz.trapmf(obstacle_left.universe, [-1, 0, 0,3, 0,5]) # "Lower", created using the fist value as being outside of range "to the left"

# Control output memberships, use tirangular even at the edges since output has limits
movement['turn_left'] = fuzz.trimf(movement.universe, [-1, -0.5, 0]) # "Lower", created using the fist value as being outside of range "to the left"
movement['forward'] = fuzz.trimf(movement.universe, [-0.5, 0, 0.5])
movement['turn_right'] = fuzz.trimf(movement.universe, [0, 0.5, 1]) # "Upper", created using the last value outside of range "to the right"

# Define fuzzy logic rules (always favor left)
rule1 = ctrl.Rule(obstacle_left['no'] & obstacle_front['no'] & obstacle_right['no'], movement['forward'])
rule2 = ctrl.Rule(obstacle_left['yes'] & obstacle_front['no'] & obstacle_right['no'], movement['forward'])
rule3 = ctrl.Rule(obstacle_left['no'] & obstacle_front['yes'] & obstacle_right['no'], movement['turn_left']) # Favor left
rule4 = ctrl.Rule(obstacle_left['no'] & obstacle_front['no'] & obstacle_right['yes'], movement['forward'])
rule5 = ctrl.Rule(obstacle_left['no'] & obstacle_front['yes'] & obstacle_right['yes'], movement['turn_left'])
rule6 = ctrl.Rule(obstacle_left['yes'] & obstacle_front['no'] & obstacle_right['yes'], movement['forward'])
rule7 = ctrl.Rule(obstacle_left['yes'] & obstacle_front['yes'] & obstacle_right['no'], movement['turn_right'])
rule8 = ctrl.Rule(obstacle_left['yes'] & obstacle_front['yes'] & obstacle_right['yes'], movement['turn_left']) # Favor left

# Create fuzzy control system
fuzzy_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8])
fuzzy_system = ctrl.ControlSystemSimulation(fuzzy_ctrl)
#------------------------------------------------------------------------------


#==============================================================================
# Get sensor readings
def get_sensor_readings():
    # Initialize
    rclpy.init()

    # Create subscriber/listener node
    node = rclpy.create_node('sensor_subscriber')

    # The function executed each time a message is received
    def listener_callback(msg):
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

    # Provide sensor values to fuzzy system
    # Left value is min value of a 105 degree cone to the left
    fuzzy_system.input['obstacle_left'] = np.min(np_sensor_data[-31:-135])
    # Front value is min value of a 60 degree cone forward
    fuzzy_system.input['obstacle_front'] = np.min(np.concatenate((np_sensor_data[-30:], np_sensor_data[0:30]), axis=None))
    # Right value is min value of a 105 degree cone to the right
    fuzzy_system.input['obstacle_right'] = np.min(np_sensor_dat[31:135])

    # Fuzzy computation
    fuzzy_system.compute()

    # Fuzzy decision on which movement should be taken
    movement_value = fuzzy_system.output['movement']

    return movement_value
#==============================================================================

#==============================================================================
def main():
    # Initialize
    rclpy.init()

    # Start taking sensor values
    # TODO: NEEDS TO BE A SEPERATE THREAD WHICH CONSTANTLY WRITES TO GLOBAL VARIABLE raw_sensor_data
    get_sensor_readings()

    while(1):
        # Create publisher node
        node = rclpy.create_node('movement_publisher')
        # Publish on command topic
        publisher = node.create_publisher(String, '/cmd_vel', 10)

        msg = String()

        # What to do every time timer goes off
        def timer_callback():
            msg.data = movement_choice()
            # Echo what the message contains
            node.get_logger().info('Publishing: "%s"' % msg.data)
            # Send message
            publisher.publish(msg)

        # Send command every 0.5 seconds
        timer_period = 0.5  # seconds
        timer = node.create_timer(timer_period, timer_callback)

        # Spin until work is complete
        rclpy.spin(node)


    # Destroy the timer attached to the node explicitly
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()
#==============================================================================

#==============================================================================
if __name__ == "__main__":
    main()
#==============================================================================
