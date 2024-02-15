# main.py
#==============================================================================
# Author: Carl Larsson
# Description: Utility functions
# Date: 15-02-2024

# This software is licensed under the MIT License
# Refer to the LICENSE file for details
#==============================================================================


#------------------------------------------------------------------------------
# Libraries

# Base libraries
import time

# rclpy libraries
import rclpy
# To reset world
from std_srvs.srv import Empty

# Project libraries
from lib import shared_variables
#------------------------------------------------------------------------------


#==============================================================================
# Reset simulation
def reset_simulation(node_array):
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
    while(shared_variables.position == None):
        pass

    # Continously check if turtlebot has made it out of the maze
    while(shared_variables.shutdown_flag != True):
        # Reset simulation once goal is reached
        if((shared_variables.position.x > 33) | (shared_variables.position.x < -33) | (shared_variables.position.y > 33) | (shared_variables.position.y < -33)):
            print("Goal reached, reseting")
            reset_world.call_async(request)
            time.sleep(5)
#==============================================================================

#==============================================================================
# Shutdown function
def shutdown_function(node_array, executor_array):
    user_input = ""

    # Wait for keyboard input q, quit or exit, then initiate shutdown
    while(1):
        user_input = input()
        if((user_input.lower() != "q") | (user_input.lower() != "quit") | (user_input.lower() != "exit")):
            break
        time.sleep(1/100)

    # Exit all forever while loops so threads can join
    shared_variables.shutdown_flag = True

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
