#! /usr/bin/env python3

# main.py
#==============================================================================
# Author: Carl Larsson
# Description: Fuzzy + reinforcement learning maze solver and graph mapping
# Date: 16-01-2024

# This software is licensed under the MIT License
# Refer to the LICENSE file for details
#==============================================================================


#------------------------------------------------------------------------------
# Base libraries
import sys
import argparse
import threading
import time

# rclpy libraries
import rclpy

# Project libraries
from lib.utility import visualize_state_map, load_state_map, save_state_map, reset_simulation, shutdown_function
from lib.data_communication import get_sensor_readings, get_coordinates_velocity
from lib.robot import robot_control
from lib.fuzzy import create_fuzzy_system
#------------------------------------------------------------------------------


#==============================================================================
# Main
def main(argv):
    # Arg parser
    parser = argparse.ArgumentParser(prog = "main.py", description = "Fuzzy + reinforcement learning maze solver and graph mapping")
    parser.add_argument('-l', required = False, type = str, help = "Load a state_map from given file")
    parser.add_argument('-s', required = False, default = "state_map.pickle", type = str, help = "Save state_map to given file")
    args = parser.parse_args(argv)

    # Load state_map
    state_map = load_state_map(args.l)

    # Initialize rclpy
    rclpy.init()
    # For destroying all nodes and executors, for a clean shutdown
    node_array = []
    executor_array = []

    # Create fuzzy control system
    fuzzy_system = create_fuzzy_system('centroid', 0)

    # Data and communication threads
    # Create thread for taking sensor values
    t1 = threading.Thread(target=get_sensor_readings, name='t1', args = (node_array, executor_array))
    # Thread for getting turtlebot coordinates and velocity
    t2 = threading.Thread(target=get_coordinates_velocity, name='t2', args = (node_array, executor_array))

    # Utility threads
    # Thread for reseting simulation, also saves state_map everytime goal is reached for safety
    t3 = threading.Thread(target=reset_simulation, name='t3', args = (node_array, args.s, state_map))
    # Thread for listening to keyboard input, once q, quit, exit or stop is entered, initiates shutdown
    t4 = threading.Thread(target=shutdown_function, name='t4', args = (node_array, executor_array))

    # Create thread for controlling robot
    t5 = threading.Thread(target=robot_control, name='t5', args = (node_array, fuzzy_system, state_map))

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

    # Save state_map
    save_state_map(args.s, state_map)

    # Show map
    visualize_state_map(state_map)

    # Only returns on shutdown
    return None
#==============================================================================

#==============================================================================
if __name__ == "__main__":
    main(sys.argv[1:])
#==============================================================================
