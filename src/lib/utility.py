# utility.py
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
import pickle
import matplotlib.pyplot as plt
import numpy as np

# rclpy libraries
import rclpy
# To reset world
from std_srvs.srv import Empty

# Project libraries
from lib import shared_variables
#------------------------------------------------------------------------------


#==============================================================================
# Plots all the states in state_map in a scatter plot
def visualize_state_map(state_map):
    # Store all state coordinates and values in arrays for easy plotting
    x = []
    y = []
    value = []

    # Add all x and y coordinates and state values to arrays to easily scatter plot them
    for key in state_map:
        x.append(state_map[key].x)
        y.append(state_map[key].y)
        value.append(state_map[key].value)

    # Set clipping on colormap so extreme outliers doesn't ruin it
    vmin = 1*np.min(np.array(value))
    vmax = 1*np.average(np.array(value))

    # Plot and colormap
    plt.scatter(x, y, c = value, cmap = 'magma', vmin=vmin, vmax=vmax)
    plt.colorbar(label='State value')

    # Set axis
    plt.xlim(-shared_variables.MAZE_BOUNDARY_COORDINATE - 1, shared_variables.MAZE_BOUNDARY_COORDINATE + 1)
    plt.ylim(-shared_variables.MAZE_BOUNDARY_COORDINATE - 1, shared_variables.MAZE_BOUNDARY_COORDINATE + 1)

    # Labels and title
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.title('Plot of all states in state_map, with color based on state value (low value good)')

    plt.grid(True)

    plt.show()

    # Nothing to return
    return None
#==============================================================================

#==============================================================================
# Load state_map from file or create empty state_map
def load_state_map(load_file):
    # If no file is provided to load state_map from, create empty one
    if(load_file == None):
        state_map = {}
        print(f"No state_map loaded, created new empty one, {len(state_map)} number of states")
    # Load state_map
    else:
        try:
            with open(load_file, 'rb') as file:
                state_map = pickle.load(file)
            print(f"Successfully loaded state_map from {load_file}, containing {len(state_map)} number of states")
        except Exception as e:
            print(f"Error loading state_map from {load_file}: {e}")
            exit(1)

    return state_map
#==============================================================================

#==============================================================================
# Save state_map to file
def save_state_map(save_file, state_map):
    # Save state_map to save_file
    try:
        with open(save_file, 'wb') as file:
            pickle.dump(state_map, file)
        print(f"state_map saved to {save_file} successfully, containing {len(state_map)} number of states.")
    except Exception as e:
        print(f"Error saving data to {save_file}: {e}")
        exit(1)

    # Nothing to return
    return None
#==============================================================================

#==============================================================================
# Checks if goal has been reached
def found_goal():
    if((shared_variables.position.x > shared_variables.MAZE_BOUNDARY_COORDINATE) |
       (shared_variables.position.x < -shared_variables.MAZE_BOUNDARY_COORDINATE) |
       (shared_variables.position.y > shared_variables.MAZE_BOUNDARY_COORDINATE) |
       (shared_variables.position.y < -shared_variables.MAZE_BOUNDARY_COORDINATE)):
        return True
    else:
        return False
#==============================================================================

#==============================================================================
# Reset simulation, and saves state_map everytime goal is reached for safety
def reset_simulation(node_array, save_file, state_map):
    # Create service reset node
    node = rclpy.create_node('service_reset')
    reset_world = node.create_client(Empty, '/reset_world')
    # Wait until reset service is ready
    reset_world.wait_for_service()
    # Reset request message
    request = Empty.Request()
    # Add node to node array for shutdown
    node_array.append(node)

    # Wait until position has a value (aka until a turtlebot position has been received)
    while(shared_variables.position == None):
        pass

    while(shared_variables.shutdown_flag == False):
        # Reset simulation once goal is reached or reset has been requested
        if((found_goal()) | (shared_variables.reset_request == True)):
            if found_goal():
                print("Goal reached, reseting")
            else:
                print("!!Reset request received, reseting!!")

            time.sleep(5)

            if found_goal():
                # Save state_map
                save_state_map(save_file, state_map)

            reset_world.call_async(request)
            time.sleep(1/10)
            shared_variables.reset_request = False

    # Only returns on shutdown
    return None
#==============================================================================

#==============================================================================
# Shutdown function
def shutdown_function(node_array, executor_array):
    # Wait for keyboard input q, quit, exit or stop, then initiate shutdown
    while(1):
        user_input = input()
        if((user_input.lower() == "q") | (user_input.lower() == "quit") | (user_input.lower() == "exit") | (user_input.lower() == "stop")):
            break
        time.sleep(1/5)

    # Exit all forever while loops so threads can join
    shared_variables.shutdown_flag = True

    time.sleep(1/2)

    # Destroy all nodes explicitly
    for node in node_array:
        node.destroy_node()

    # Shut down all executors
    for executor in executor_array:
        executor.shutdown()

    time.sleep(1/2)

    # Shutdown and free all rclpy resources
    rclpy.shutdown()

    print("\n Shutdown succesful, have a nice day!\n")

    # Only returns on shutdown
    return None
#==============================================================================
