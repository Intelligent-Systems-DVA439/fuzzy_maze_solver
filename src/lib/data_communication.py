
# data_communication.py
#==============================================================================
# Author: Carl Larsson
# Description: data and communication functions
# Date: 15-02-2024

# This software is licensed under the MIT License
# Refer to the LICENSE file for details
#==============================================================================


#------------------------------------------------------------------------------
# Libraries

# rclpy libraries
import rclpy
# laser scan message struct
from sensor_msgs.msg import LaserScan
# Get turtlebot position
from nav_msgs.msg import Odometry

# Project libraries
from lib import shared_variables
#------------------------------------------------------------------------------


#==============================================================================
# Get sensor readings
def get_sensor_readings(node_array, executor_array):
    # Create sensor subscriber/listener node
    node = rclpy.create_node('sensor_subscriber')

    # The function executed each time a message is received
    def sensor_listener_callback(msg):
        # Store data in a global array
        shared_variables.raw_sensor_data = msg.ranges

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
# Get turtlebot coordinates
def get_coordinates(node_array, executor_array):
    # Create subscriber node for coordinates of turtlebot
    node = rclpy.create_node('coordinates_subscriber')

    # The function executed each time a message is received
    def coordinate_callback(msg):
        # Store data in a global variable
        shared_variables.position = msg.pose.pose.position

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
