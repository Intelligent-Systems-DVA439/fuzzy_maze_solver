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
#------------------------------------------------------------------------------

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

# Control output memberships
movement['turn_left'] = fuzz.trimf(movement.universe, [-1, -1, 0])
movement['forward'] = fuzz.trimf(movement.universe, [-0.5, 0, 0.5])
movement['turn_right'] = fuzz.trimf(movement.universe, [0, 1, 1])

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

#==============================================================================


#==============================================================================
# Maze solver, decide movement decision
def movement_choice():
    # Get sensor values (percept)
    obstacle_left_value, obstacle_front_value, obstacle_right_value = get_sensor_readings()

    # Provide sensor values to fuzzy system
    fuzzy_system.input['obstacle_left'] = obstacle_left_value
    fuzzy_system.input['obstacle_front'] = obstacle_front_value
    fuzzy_system.input['obstacle_right'] = obstacle_right_value

    # Fuzzy computation
    fuzzy_system.compute()

    # Fuzzy decision on which movement should be taken
    movement_value = fuzzy_system.output['movement']
#==============================================================================

#==============================================================================
def main():

#==============================================================================

#==============================================================================
if __name__ == "__main__":
    main()
#==============================================================================

