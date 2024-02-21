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
import threading
import time

# rclpy libraries
import rclpy

# Project libraries
from lib.utility import reset_simulation, shutdown_function
from lib.data_communication import get_sensor_readings, get_coordinates_velocity
from lib.robot import robot_control, Range
from lib.fuzzy import create_fuzzy_system
#------------------------------------------------------------------------------


#==============================================================================
# Main
def main():
    # Initialize rclpy
    rclpy.init()
    # For destroying all nodes and executors, for a clean shutdown
    node_array = []
    executor_array = []
    # For normalization
    sensor = Range(0, 3.51)
    linear = Range(-0.26, 0.26)
    angular = Range(-1.82, 1.82)

    # Create fuzzy control system
    fuzzy_system = create_fuzzy_system('centroid', 0)

    # Data and communication threads
    # Create thread for taking sensor values
    t1 = threading.Thread(target=get_sensor_readings, name='t1', args = (node_array, executor_array))
    # Thread for getting turtlebot coordinates and velocity
    t2 = threading.Thread(target=get_coordinates_velocity, name='t2', args = (node_array, executor_array))

    # Utility threads
    # Thread for reseting once goal is reached
    # Threading requires the args to be sent as a tuple, hence the (arg1,) despite only sending 1 arg
    t3 = threading.Thread(target=reset_simulation, name='t3', args = (node_array,))
    # Thread for listening to keyboard input, once q, quit or exit is entered, initiates shutdown
    t4 = threading.Thread(target=shutdown_function, name='t4', args = (node_array, executor_array))

    # Create thread for controlling robot
    t5 = threading.Thread(target=robot_control, name='t5', args = (node_array, fuzzy_system, sensor, linear, angular))

    # Start threads
    t1.start()
    t2.start()
    time.sleep(1/10)
    t3.start()
    t4.start()
    t5.start()

    # Close threads once completed
    t1.join()
    t2.join()
    t3.join()
    t4.join()
    t5.join()

    # Only returns on shutdown
    return None
#==============================================================================

#==============================================================================
if __name__ == "__main__":
    main()
#==============================================================================
