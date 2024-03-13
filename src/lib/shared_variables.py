# shared_variables.py
#==============================================================================
# Author: Carl Larsson
# Description: shared/global variables
# Date: 15-02-2024

# This software is licensed under the MIT License
# Refer to the LICENSE file for details
#==============================================================================


#------------------------------------------------------------------------------
# Libraries

# Base libraries
import threading
#------------------------------------------------------------------------------


#------------------------------------------------------------------------------
# Global variables

# Global variable to store turtlebot coordinates/position
position = None
# Global variable to store turtlebot velocity (linear and angular)
velocity = None
# Global array to receive sensor data (distance/range) from turtlebot
raw_sensor_data = [-1 for x in range(360)]
# Shutdown flag
shutdown_flag = False
# Request to reset simulation
reset_request = False
# Maze boundry coordinates (assumes maze is centered and square)
MAZE_BOUNDARY_COORDINATE = 8.8
# Mutex for securing access to state_map
sm_mutex = threading.Semaphore()
#------------------------------------------------------------------------------
