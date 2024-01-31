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
import time
import numpy as np
from numpy import inf
import skfuzzy as fuzz
from skfuzzy import control as ctrl

import rclpy
# laser scan message struct
from sensor_msgs.msg import LaserScan
# Movement message struct
from geometry_msgs.msg import Twist
# Get turtlebot position
from nav_msgs.msg import Odometry
# To reset world
from std_srvs.srv import Empty
#------------------------------------------------------------------------------


#------------------------------------------------------------------------------
# Global variables

# Global variable to store turtlebot coordinates/position
position = None
# Global value to receive sensor data (distance/range) from turtlebot
raw_sensor_data = [-1 for x in range(360)]
# Shutdown flag
shutdown_flag = False
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
left_sensor['close'] = fuzz.trapmf(left_sensor.universe, [-math.inf, 0, 0.2, 0.4]) # "Lower", created using the fist value as being outside of range "to the left"
left_sensor['medium'] = fuzz.trimf(left_sensor.universe, [0.2, 0.4, 1.4])
left_sensor['far'] = fuzz.trapmf(left_sensor.universe, [0.4, 1.4, 3.5, math.inf]) # "Upper", created using the last value outside of range "to the right"

front_sensor['close'] = fuzz.trapmf(front_sensor.universe, [-math.inf, 0, 0.15, 0.3]) # "Lower", created using the fist value as being outside of range "to the left"
front_sensor['medium'] = fuzz.trimf(front_sensor.universe, [0.15, 0.3, 1])
front_sensor['far'] = fuzz.trapmf(front_sensor.universe, [0.3, 1, 3.5, math.inf]) # "Upper", created using the last value outside of range "to the right"

right_sensor['close'] = fuzz.trapmf(right_sensor.universe, [-math.inf, 0, 0.2, 0.4]) # "Lower", created using the fist value as being outside of range "to the left"
right_sensor['medium'] = fuzz.trimf(right_sensor.universe, [0.2, 0.4, 1.4])
right_sensor['far'] = fuzz.trapmf(right_sensor.universe, [0.4, 1.4, 3.5, math.inf]) # "Upper", created using the last value outside of range "to the right"

# Control output memberships, use tirangular even at the edges since output has limits
# Linear
linear_movement['linear_reverse'] = fuzz.trimf(linear_movement.universe, [-0.26, -0.15, -0.04])
linear_movement['linear_stop'] = fuzz.trimf(linear_movement.universe, [-0.04, 0, 0.04])
linear_movement['linear_forward'] = fuzz.trimf(linear_movement.universe, [0.04, 0.15, 0.26])

# Angular
# z-aix is positive counter clockwise, and negative clockwise (viewed from above)
angular_movement['angular_right_fast'] = fuzz.trimf(angular_movement.universe, [-1.82, -1.32, -0.82])
angular_movement['angular_right_slow'] = fuzz.trimf(angular_movement.universe, [-1, -0.5, 0])
angular_movement['angular_stop'] = fuzz.trimf(angular_movement.universe, [-0.1, 0, 0.1]) 
angular_movement['angular_left_slow'] = fuzz.trimf(angular_movement.universe, [0, 0.5, 1])
angular_movement['angular_left_fast'] = fuzz.trimf(angular_movement.universe, [0.82, 1.32, 1.82])


# Define fuzzy logic rules
# Linear
rule1 = ctrl.Rule(front_sensor['close'], linear_movement['linear_reverse'])
rule2 = ctrl.Rule(front_sensor['medium'], linear_movement['linear_stop'])
rule3 = ctrl.Rule(front_sensor['far'], linear_movement['linear_forward'])

# Angular
rule4 = ctrl.Rule(left_sensor['far'] & front_sensor['far'] & right_sensor['far'], angular_movement['angular_stop'])
rule5 = ctrl.Rule(left_sensor['far'] & front_sensor['far'] & right_sensor['medium'], angular_movement['angular_left_slow'])
rule6 = ctrl.Rule(left_sensor['far'] & front_sensor['far'] & right_sensor['close'], angular_movement['angular_left_fast'])
rule7 = ctrl.Rule(left_sensor['far'] & front_sensor['medium'] & right_sensor['far'], angular_movement['angular_left_slow']) # Favor left
rule8 = ctrl.Rule(left_sensor['far'] & front_sensor['medium'] & right_sensor['medium'], angular_movement['angular_left_slow'])
rule9 = ctrl.Rule(left_sensor['far'] & front_sensor['medium'] & right_sensor['close'], angular_movement['angular_left_fast'])
rule10 = ctrl.Rule(left_sensor['far'] & front_sensor['close'] & right_sensor['far'], angular_movement['angular_left_fast']) # Favor left
rule11 = ctrl.Rule(left_sensor['far'] & front_sensor['close'] & right_sensor['medium'], angular_movement['angular_left_fast'])
rule12 = ctrl.Rule(left_sensor['far'] & front_sensor['close'] & right_sensor['close'], angular_movement['angular_left_fast'])
rule13 = ctrl.Rule(left_sensor['medium'] & front_sensor['far'] & right_sensor['far'], angular_movement['angular_right_slow'])
rule14 = ctrl.Rule(left_sensor['medium'] & front_sensor['far'] & right_sensor['medium'], angular_movement['angular_stop'])
rule15 = ctrl.Rule(left_sensor['medium'] & front_sensor['far'] & right_sensor['close'], angular_movement['angular_left_fast'])
rule16 = ctrl.Rule(left_sensor['medium'] & front_sensor['medium'] & right_sensor['far'], angular_movement['angular_right_slow'])
rule17 = ctrl.Rule(left_sensor['medium'] & front_sensor['medium'] & right_sensor['medium'], angular_movement['angular_left_slow']) # Favor left
rule18 = ctrl.Rule(left_sensor['medium'] & front_sensor['medium'] & right_sensor['close'], angular_movement['angular_left_fast'])
rule19 = ctrl.Rule(left_sensor['medium'] & front_sensor['close'] & right_sensor['far'], angular_movement['angular_right_fast'])
rule20 = ctrl.Rule(left_sensor['medium'] & front_sensor['close'] & right_sensor['medium'], angular_movement['angular_left_fast']) # Favor left
rule21 = ctrl.Rule(left_sensor['medium'] & front_sensor['close'] & right_sensor['close'], angular_movement['angular_left_fast'])
rule22 = ctrl.Rule(left_sensor['close'] & front_sensor['far'] & right_sensor['far'], angular_movement['angular_right_fast'])
rule23 = ctrl.Rule(left_sensor['close'] & front_sensor['far'] & right_sensor['medium'], angular_movement['angular_right_fast'])
rule24 = ctrl.Rule(left_sensor['close'] & front_sensor['far'] & right_sensor['close'], angular_movement['angular_stop'])
rule25 = ctrl.Rule(left_sensor['close'] & front_sensor['medium'] & right_sensor['far'], angular_movement['angular_right_fast'])
rule26 = ctrl.Rule(left_sensor['close'] & front_sensor['medium'] & right_sensor['medium'], angular_movement['angular_right_fast'])
rule27 = ctrl.Rule(left_sensor['close'] & front_sensor['medium'] & right_sensor['close'], angular_movement['angular_stop'])
rule28 = ctrl.Rule(left_sensor['close'] & front_sensor['close'] & right_sensor['far'], angular_movement['angular_right_fast'])
rule29 = ctrl.Rule(left_sensor['close'] & front_sensor['close'] & right_sensor['medium'], angular_movement['angular_right_fast'])
rule30 = ctrl.Rule(left_sensor['close'] & front_sensor['close'] & right_sensor['close'], angular_movement['angular_left_fast']) # Favor left


# Create fuzzy control system
fuzzy_ctrl = ctrl.ControlSystem([
    rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10,
    rule11, rule12, rule13, rule14, rule15, rule16, rule17, rule18, rule19,
    rule20, rule21, rule22, rule23, rule24, rule25, rule26, rule27, rule28,
    rule29, rule30
    ])
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

    # Create new executor since only one can run on the global
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    # Spin until work is complete
    rclpy.spin(node, executor)

    # Shut down executor
    executor.shutdown()
    # Destroy the node explicitly
    node.destroy_node()
#==============================================================================

#==============================================================================
# Maze solver, decide which movement should be taken
def movement_choice():
    # Get sensor values (percept)
    np_sensor_data = np.array(raw_sensor_data)

    # Stuck until sensor has been initialized and sensor values have been received
    while(np.all(np_sensor_data == -1)):
        np_sensor_data = np.array(raw_sensor_data)

    # Set all inf values to max value since average is calculated later
    np_sensor_data[np_sensor_data == inf] = 3.5

    # Provide sensor values to fuzzy system
    # Lidar values go counter clockwise and start infront of the robot
    # Left value is mean value of a 80 degree cone to the left
    fuzzy_system.input['left_sensor'] = np.mean(np_sensor_data[10:80])
    # Front value is min value of a 20 degree cone forward
    fuzzy_system.input['front_sensor'] = np.min(np.concatenate((np_sensor_data[-20:], np_sensor_data[0:20]), axis=None))
    # Right value is mean value of a 80 degree cone to the right
    fuzzy_system.input['right_sensor'] = np.mean(np_sensor_data[-80:-10])
    #print("\nL: ",np.mean(np_sensor_data[10:80]),", F: ",np.min(np.concatenate((np_sensor_data[-20:], np_sensor_data[0:20]), axis=None)),", R: ",np.mean(np_sensor_data[-80:-10]),"\n")

    # Fuzzy computation
    fuzzy_system.compute()

    # Fuzzy decision on which movement should be taken
    linear_value = fuzzy_system.output['linear']
    angular_value = fuzzy_system.output['angular']

    return linear_value, angular_value
#==============================================================================

#==============================================================================
# Controls turtlebot in gazebo
def robot_control():
    global shutdown_flag

    # Create publisher node
    # https://www.youtube.com/watch?v=yEwi1__NJrE
    node = rclpy.create_node('movement_publisher')
    # Publish on command topic
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    # set message to correct struct type
    msg = Twist()

    while(shutdown_flag != True):
        # Decide which linear and angular movement should be taken
        linear_value, angular_value = movement_choice()
        msg.linear.x = linear_value
        msg.angular.z = angular_value
        # Send message
        publisher.publish(msg)

    # Destroy node explicitly
    node.destroy_node()
#==============================================================================

#==============================================================================
# Get turtlebot coordinates
def get_coordinates():
    # Create subscriber node for coordinates of turtlebot
    node = rclpy.create_node('coordinates_subscriber')

    # The function executed each time a message is received
    def coordinate_callback(msg):
        # Required so the variable can be changed
        global position

        # Store data in a global variable
        position = msg.pose.pose.position

    # Subscribe to the topic for the turtlebot coordinates
    subscription = node.create_subscription(Odometry, 'odom', coordinate_callback, 10)
    subscription  # prevent unused variable warning

    # Create new executor since only one can run on the global
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    # Spin until work is complete
    rclpy.spin(node, executor)

    # Shut down executor
    executor.shutdown()
    # Destroy the node explicitly
    node.destroy_node()
#==============================================================================

#==============================================================================
# Reset simulation
def reset_simulation():
    global shutdown_flag

    # Create service reset node
    node = rclpy.create_node('service_reset')
    reset_world = node.create_client(Empty, '/reset_world')
    # Wait until reset service is ready
    reset_world.wait_for_service()
    # Reset request
    request = Empty.Request()

    # Wait until position has a value (aka until a turtlebot position has been received)
    while(position == None):
        pass

    # Continously check if turtlebot has made it out of the maze
    while(shutdown_flag != True):
        # Reset simulation once goal is reached
        if((position.x > 10) | (position.x < -10) | (position.y > 10) | (position.y < -10)):
            print("Goal reached, reseting")
            reset_world.call_async(request)


    # Shutdown the ROS node
    node.destroy_node()
#==============================================================================

#==============================================================================
# Shutdown function
def shutdown_function():
    global shutdown_flag
    user_input = ""

    while(1):
        user_input = input()
        if((user_input.lower() != "q") | (user_input.lower() != "quit") | (user_input.lower() != "exit")):
            break
        time.sleep(1/100)

    shutdown_flag = True
    # Incase exeptions occur
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
    # Thread for reseting once goal is reached
    t3 = threading.Thread(target=reset_simulation, name='t3')
    # Thread for getting turtlebot coordinates
    t4 = threading.Thread(target=get_coordinates, name='t4')
    # Thread for listening to keyboard input, once q, quit or exit is entered, initiates shutdown
    t5 = threading.Thread(target=shutdown_function, name='t5')

    # Start threads
    t1.start()
    t2.start()
    t3.start()
    t4.start()
    t5.start()

    # Close threads once completed
    t1.join()
    t2.join()
    t3.join()
    t4.join()
    t5.join()
#==============================================================================

#==============================================================================
if __name__ == "__main__":
    main()
#==============================================================================
