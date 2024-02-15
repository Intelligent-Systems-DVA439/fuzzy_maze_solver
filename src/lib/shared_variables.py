# shared_variables.py
#==============================================================================
# Author: Carl Larsson
# Description: shared/global variables
# Date: 15-02-2024

# This software is licensed under the MIT License
# Refer to the LICENSE file for details
#==============================================================================


#------------------------------------------------------------------------------
# Global variables

# Global variable to store turtlebot coordinates/position
position = None
# Global array to receive sensor data (distance/range) from turtlebot
raw_sensor_data = [-1 for x in range(360)]
# Shutdown flag
shutdown_flag = False
#------------------------------------------------------------------------------