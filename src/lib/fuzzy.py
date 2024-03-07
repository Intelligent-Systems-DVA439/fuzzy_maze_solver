# fuzzy.py
#==============================================================================
# Author: Carl Larsson
# Description: Fuzzy functions
# Date: 15-02-2024

# This software is licensed under the MIT License
# Refer to the LICENSE file for details
#==============================================================================


#------------------------------------------------------------------------------
# Libraries

# Base libraries
import math
from enum import Enum

# Functional libraries
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt
#------------------------------------------------------------------------------


#==============================================================================
# Create fuzzy system
def create_fuzzy_system(defuzzy_method = 'centroid', visualize_memberships = 0, resolution = 0.0001):
    # Fuzzy variables
    # Define fuzzy input variables (sensors)
    # Normalized
    left_sensor = ctrl.Antecedent(np.arange(0, 1+resolution, resolution,), 'left_sensor')
    front_sensor = ctrl.Antecedent(np.arange(0, 1+resolution, resolution,), 'front_sensor')
    right_sensor = ctrl.Antecedent(np.arange(0, 1+resolution, resolution,), 'right_sensor')

    # Define fuzzy output variable (control signal)
    # Normalized
    linear_movement = ctrl.Consequent(np.arange(-1, 1+resolution, resolution,), 'linear')
    angular_movement = ctrl.Consequent(np.arange(-1, 1+resolution, resolution,), 'angular')


    # Side sensor enum
    class Side_sensor(Enum):
        CLOSE = 0.05 # Affects jittering towards wall
        MEDIUM = 0.15 # Affects jittering towards wall
        FAR = 0.55 # Affects cornering and how close to corner it goes, aswell as willingness to enter openings
    # Define membership functions, using triangular and trapezoidal memberships
    # Normalized sensor readings memberships
    left_sensor['close'] = fuzz.trapmf(left_sensor.universe, [-math.inf, 0, Side_sensor.CLOSE.value, Side_sensor.MEDIUM.value]) # "Lower", created using the fist value as being outside of range "to the left"
    left_sensor['medium'] = fuzz.trimf(left_sensor.universe, [Side_sensor.CLOSE.value, Side_sensor.MEDIUM.value, Side_sensor.FAR.value])
    left_sensor['far'] = fuzz.trapmf(left_sensor.universe, [Side_sensor.MEDIUM.value, Side_sensor.FAR.value, 1, math.inf]) # "Upper", created using the last value outside of range "to the right"

    # Front sensor enum
    class Front_sensor(Enum):
        CLOSE = 0.04
        MEDIUM = 0.08
        FAR = 0.28
    front_sensor['close'] = fuzz.trapmf(front_sensor.universe, [-math.inf, 0, Front_sensor.CLOSE.value, Front_sensor.MEDIUM.value]) # "Lower", created using the fist value as being outside of range "to the left"
    front_sensor['medium'] = fuzz.trimf(front_sensor.universe, [Front_sensor.CLOSE.value, Front_sensor.MEDIUM.value, Front_sensor.FAR.value])
    front_sensor['far'] = fuzz.trapmf(front_sensor.universe, [Front_sensor.MEDIUM.value, Front_sensor.FAR.value, 1, math.inf]) # "Upper", created using the last value outside of range "to the right"

    right_sensor['close'] = fuzz.trapmf(right_sensor.universe, [-math.inf, 0, Side_sensor.CLOSE.value, Side_sensor.MEDIUM.value]) # "Lower", created using the fist value as being outside of range "to the left"
    right_sensor['medium'] = fuzz.trimf(right_sensor.universe, [Side_sensor.CLOSE.value, Side_sensor.MEDIUM.value, Side_sensor.FAR.value])
    right_sensor['far'] = fuzz.trapmf(right_sensor.universe, [Side_sensor.MEDIUM.value, Side_sensor.FAR.value, 1, math.inf]) # "Upper", created using the last value outside of range "to the right"

    # Linear movement enum
    class Linear(Enum):
        REVERSE = 0.58
        STOP = 0.15
        FORWARD = 0.58
    # Normalized control output memberships, use tirangular even at the edges since output has limits
    # Linear
    linear_movement['linear_reverse'] = fuzz.trimf(linear_movement.universe, [-1, -Linear.REVERSE.value, -Linear.STOP.value])
    linear_movement['linear_stop'] = fuzz.trimf(linear_movement.universe, [-Linear.STOP.value, 0, Linear.STOP.value])
    linear_movement['linear_forward'] = fuzz.trimf(linear_movement.universe, [Linear.STOP.value, Linear.FORWARD.value, 1])

    # Angular movement enum
    class Angular(Enum):
        STOP = 0.05
        SLOW_LOWER = 0
        SLOW = 0.28
        SLOW_UPPER = 0.55
        FAST_LOWER = 0.45
        FAST = 0.73
    # Angular
    # z-aix is positive counter clockwise, and negative clockwise (viewed from above)
    angular_movement['angular_left_fast'] = fuzz.trimf(angular_movement.universe, [Angular.FAST_LOWER.value, Angular.FAST.value, 1])
    angular_movement['angular_left_slow'] = fuzz.trimf(angular_movement.universe, [Angular.SLOW_LOWER.value, Angular.SLOW.value, Angular.SLOW_UPPER.value])
    angular_movement['angular_stop'] = fuzz.trimf(angular_movement.universe, [-Angular.STOP.value, 0, Angular.STOP.value])
    angular_movement['angular_right_slow'] = fuzz.trimf(angular_movement.universe, [-Angular.SLOW_UPPER.value, -Angular.SLOW.value, -Angular.SLOW_LOWER.value])
    angular_movement['angular_right_fast'] = fuzz.trimf(angular_movement.universe, [-1, -Angular.FAST.value, -Angular.FAST_LOWER.value])


    # Visualize memberships
    if visualize_memberships:
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

    # Define defuzzification method
    fuzzy_system.defuzzify_method = defuzzy_method

    return fuzzy_system
#==============================================================================
