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
# Base libraries
import math
import threading
import time

# Functional libraries
import numpy as np
from numpy import inf
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

# Robot libraries
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
# Global array to receive sensor data (distance/range) from turtlebot
raw_sensor_data = [-1 for x in range(360)]
# Shutdown flag
shutdown_flag = False
#------------------------------------------------------------------------------


#==============================================================================
# Create fuzzy system
def create_fuzzy_system(defuzzy_method = 'centroid', visualize_memberships = 0):
    # Fuzzy variables
    # Define fuzzy input variables (sensors)
    # Normalized
    resolution = 0.0001
    left_sensor = ctrl.Antecedent(np.arange(0, 1+resolution, resolution,), 'left_sensor')
    front_sensor = ctrl.Antecedent(np.arange(0, 1+resolution, resolution,), 'front_sensor')
    right_sensor = ctrl.Antecedent(np.arange(0, 1+resolution, resolution,), 'right_sensor')

    # Define fuzzy output variable (control signal)
    # Normalized
    linear_movement = ctrl.Consequent(np.arange(-1, 1+resolution, resolution,), 'linear')
    angular_movement = ctrl.Consequent(np.arange(-1, 1+resolution, resolution,), 'angular')


    # Define membership functions, using triangular and trapezoidal memberships
    # Normalized sensor readings memberships
    left_sensor['close'] = fuzz.trapmf(left_sensor.universe, [-math.inf, 0, 0.114, 0.228]) # "Lower", created using the fist value as being outside of range "to the left"
    left_sensor['medium'] = fuzz.trimf(left_sensor.universe, [0.114, 0.228, 0.798])
    left_sensor['far'] = fuzz.trapmf(left_sensor.universe, [0.228, 0.798, 1, math.inf]) # "Upper", created using the last value outside of range "to the right"

    front_sensor['close'] = fuzz.trapmf(front_sensor.universe, [-math.inf, 0, 0.043, 0.085]) # "Lower", created using the fist value as being outside of range "to the left"
    front_sensor['medium'] = fuzz.trimf(front_sensor.universe, [0.043, 0.085, 0.285])
    front_sensor['far'] = fuzz.trapmf(front_sensor.universe, [0.085, 0.285, 1, math.inf]) # "Upper", created using the last value outside of range "to the right"

    right_sensor['close'] = fuzz.trapmf(right_sensor.universe, [-math.inf, 0, 0.114, 0.228]) # "Lower", created using the fist value as being outside of range "to the left"
    right_sensor['medium'] = fuzz.trimf(right_sensor.universe, [0.114, 0.228, 0.798])
    right_sensor['far'] = fuzz.trapmf(right_sensor.universe, [0.228, 0.798, 1, math.inf]) # "Upper", created using the last value outside of range "to the right"

    # Normalized control output memberships, use tirangular even at the edges since output has limits
    # Linear
    linear_movement['linear_reverse'] = fuzz.trimf(linear_movement.universe, [-1, -0.577, -0.154])
    linear_movement['linear_stop'] = fuzz.trimf(linear_movement.universe, [-0.154, 0, 0.154])
    linear_movement['linear_forward'] = fuzz.trimf(linear_movement.universe, [0.154, 0.577, 1])

    # Angular
    # z-aix is positive counter clockwise, and negative clockwise (viewed from above)
    angular_movement['angular_right_fast'] = fuzz.trimf(angular_movement.universe, [-1, -0.725, -0.451])
    angular_movement['angular_right_slow'] = fuzz.trimf(angular_movement.universe, [-0.550, -0.275, 0])
    angular_movement['angular_stop'] = fuzz.trimf(angular_movement.universe, [-0.055, 0, 0.055]) 
    angular_movement['angular_left_slow'] = fuzz.trimf(angular_movement.universe, [0, 0.275, 0.550])
    angular_movement['angular_left_fast'] = fuzz.trimf(angular_movement.universe, [0.451, 0.725, 1])


    # Visualize memberships
    if(visualize_memberships != 0):
        left_sensor.view()
        front_sensor.view()
        right_sensor.view()
        linear_movement.view()
        angular_movement.view()
        plt.show()


    # Define fuzzy logic rules
    # Linear
    rule1 = ctrl.Rule(front_sensor['close'], linear_movement['linear_reverse'])
    rule2 = ctrl.Rule(front_sensor['medium'], linear_movement['linear_stop'])
    rule3 = ctrl.Rule(front_sensor['far'], linear_movement['linear_forward'])

    # Angular
    rule4 = ctrl.Rule(left_sensor['far'] & front_sensor['far'] & right_sensor['far'], angular_movement['angular_left_slow']) # Favor Left (Required for the turtlebot to take left in T crossings)
    rule5 = ctrl.Rule(left_sensor['far'] & front_sensor['far'] & right_sensor['medium'], angular_movement['angular_left_slow']) #?
    rule6 = ctrl.Rule(left_sensor['far'] & front_sensor['far'] & right_sensor['close'], angular_movement['angular_left_fast'])
    rule7 = ctrl.Rule(left_sensor['far'] & front_sensor['medium'] & right_sensor['far'], angular_movement['angular_left_slow']) # Favor left
    rule8 = ctrl.Rule(left_sensor['far'] & front_sensor['medium'] & right_sensor['medium'], angular_movement['angular_left_slow'])
    rule9 = ctrl.Rule(left_sensor['far'] & front_sensor['medium'] & right_sensor['close'], angular_movement['angular_left_fast'])
    rule10 = ctrl.Rule(left_sensor['far'] & front_sensor['close'] & right_sensor['far'], angular_movement['angular_left_fast']) # Favor left
    rule11 = ctrl.Rule(left_sensor['far'] & front_sensor['close'] & right_sensor['medium'], angular_movement['angular_left_fast'])
    rule12 = ctrl.Rule(left_sensor['far'] & front_sensor['close'] & right_sensor['close'], angular_movement['angular_left_fast'])
    rule13 = ctrl.Rule(left_sensor['medium'] & front_sensor['far'] & right_sensor['far'], angular_movement['angular_right_slow']) #?
    rule14 = ctrl.Rule(left_sensor['medium'] & front_sensor['far'] & right_sensor['medium'], angular_movement['angular_stop'])
    rule15 = ctrl.Rule(left_sensor['medium'] & front_sensor['far'] & right_sensor['close'], angular_movement['angular_left_fast'])
    rule16 = ctrl.Rule(left_sensor['medium'] & front_sensor['medium'] & right_sensor['far'], angular_movement['angular_right_slow']) #?
    rule17 = ctrl.Rule(left_sensor['medium'] & front_sensor['medium'] & right_sensor['medium'], angular_movement['angular_left_slow']) # Favor left
    rule18 = ctrl.Rule(left_sensor['medium'] & front_sensor['medium'] & right_sensor['close'], angular_movement['angular_left_fast'])
    rule19 = ctrl.Rule(left_sensor['medium'] & front_sensor['close'] & right_sensor['far'], angular_movement['angular_right_fast']) #?
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

    # Change defuzzification method
    fuzzy_system.defuzzify_method = defuzzy_method

    return fuzzy_system
#==============================================================================

#==============================================================================
# Get sensor readings
def get_sensor_readings(node_array, executor_array):
    # Create sensor subscriber/listener node
    node = rclpy.create_node('sensor_subscriber')

    # The function executed each time a message is received
    def sensor_listener_callback(msg):
        # Required so the array can be changed
        global raw_sensor_data

        # Store data in a global array
        raw_sensor_data = msg.ranges

    # Subscribe to the topic for the lidar/LaserScan
    # ros2 topic info /scan
    # https://www.youtube.com/watch?v=RFNNsDI2b6c&t=1s
    subscription = node.create_subscription(LaserScan, '/scan', sensor_listener_callback, 10)
    subscription  # prevent unused variable warning
    # Add node to node array for shutdown
    node_array.append(node)

    # Create new executor since only one can run on the global
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    # Add executor to executor array for shutdown
    executor_array.append(executor)

    # Spin until work is complete
    rclpy.spin(node, executor)
#==============================================================================

#==============================================================================
# Maze solver, decide which movement should be taken
def movement_choice(fuzzy_system, min_sensor_value, max_sensor_value, min_linear, max_linear, min_angular, max_angular):
    global raw_sensor_data

    # Get sensor values (percept)
    np_sensor_data = np.array(raw_sensor_data)

    # Stuck until sensor has been initialized and sensor values have been received
    while(np.all(np_sensor_data == -1)):
        np_sensor_data = np.array(raw_sensor_data)

    # Set all inf values to max value since average is calculated later
    np_sensor_data[np_sensor_data == inf] = 3.5

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
    global shutdown_flag

    # Create publisher node
    # https://www.youtube.com/watch?v=yEwi1__NJrE
    node = rclpy.create_node('movement_publisher')
    # Publish on command topic
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    # Add node to node array for shutdown
    node_array.append(node)

    # set message to correct struct type
    msg = Twist()

    while(shutdown_flag != True):
        # Decide which linear and angular movement should be taken
        linear_value, angular_value = movement_choice(fuzzy_system, min_sensor_value, max_sensor_value, min_linear, max_linear, min_angular, max_angular)
        msg.linear.x = linear_value
        msg.angular.z = angular_value
        # Send message
        publisher.publish(msg)
#==============================================================================

#==============================================================================
# Get turtlebot coordinates
def get_coordinates(node_array, executor_array):
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
    # Add node to node array for shutdown
    node_array.append(node)

    # Create new executor since only one can run on the global
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    # Add executor to executor array for shutdown
    executor_array.append(executor)

    # Spin until work is complete
    rclpy.spin(node, executor)
#==============================================================================

#==============================================================================
# Reset simulation
def reset_simulation(node_array):
    global shutdown_flag

    # Create service reset node
    node = rclpy.create_node('service_reset')
    reset_world = node.create_client(Empty, '/reset_world')
    # Wait until reset service is ready
    reset_world.wait_for_service()
    # Reset request
    request = Empty.Request()
    # Add node to node array for shutdown
    node_array.append(node)

    # Wait until position has a value (aka until a turtlebot position has been received)
    while(position == None):
        pass

    # Continously check if turtlebot has made it out of the maze
    while(shutdown_flag != True):
        # Reset simulation once goal is reached
        if((position.x > 33) | (position.x < -33) | (position.y > 33) | (position.y < -33)):
            print("Goal reached, reseting")
            reset_world.call_async(request)
#==============================================================================

#==============================================================================
# Shutdown function
def shutdown_function(node_array, executor_array):
    global shutdown_flag
    user_input = ""

    # Wait for keyboard input q, quit or exit, then initiate shutdown
    while(1):
        user_input = input()
        if((user_input.lower() != "q") | (user_input.lower() != "quit") | (user_input.lower() != "exit")):
            break
        time.sleep(1/100)

    # Exit all forever while loops so threads can join
    shutdown_flag = True

    time.sleep(1)

    # Destroy all nodes explicitly
    for node in node_array:
        node.destroy_node()

    # Shut down all executors
    for executor in executor_array:
        executor.shutdown()

    time.sleep(1)

    # Shutdown and free all rclpy resources
    rclpy.shutdown()

    print("\n Shutdown succesful, have a nice day!\n")
#==============================================================================

#==============================================================================
# Main
def main():
    # Initialize rclpy
    rclpy.init()
    # For destroying all nodes and executors, for a clean shutdown
    node_array = []
    executor_array = []
    # For normalization
    min_sensor_value = 0
    max_sensor_value = 3.51
    min_linear = -0.26
    max_linear = 0.26
    min_angular = -1.82
    max_angular = 1.82

    # Create fuzzy control syste
    fuzzy_system = create_fuzzy_system('centroid', 0)

    # Create thread for taking sensor values
    t1 = threading.Thread(target=get_sensor_readings, name='t1', args = (node_array, executor_array))
    # Create thread for controling robot
    t2 = threading.Thread(target=robot_control, name='t2', args = (node_array, fuzzy_system, min_sensor_value, max_sensor_value, min_linear, max_linear, min_angular, max_angular))
    # Thread for reseting once goal is reached
    t3 = threading.Thread(target=reset_simulation, name='t3', args = (node_array,))
    # Thread for getting turtlebot coordinates
    t4 = threading.Thread(target=get_coordinates, name='t4', args = (node_array, executor_array))
    # Thread for listening to keyboard input, once q, quit or exit is entered, initiates shutdown
    t5 = threading.Thread(target=shutdown_function, name='t5', args = (node_array, executor_array))

    # Start threads
    t1.start()
    t2.start()
    t3.start()
    t4.start()
    time.sleep(1)
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
