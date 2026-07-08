#!/usr/bin/env python3

# Copyright (c) 2020, Hugo Costelha
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
# * Neither the name of the Player Project nor the names of its contributors
#     may be used to endorse or promote products derived from this software
#     without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE 4 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

'''@package docstring
Global parameters for an application of a State Machine for high-level robotic
tasks control.
'''

# ROS related modules
from geometry_msgs.msg import Pose2D

# Our modules


# Other modules
from numpy import pi

''' Global variables/constants
    Use this mainly for constants and variables that are used everywhere,
    otheriwse you should use the input_keys and output_keys of the states.
    The use of this variables assume a single-threaded execution, otherwise
    you need to use locks to prevent simultaneous acess.
 '''
robot_name = 'robot_0'
MIN_POWER_LEVEL = 0.20  # Minimum power level before recharging
MAX_POWER_LEVEL = 1.00  # Minimum power level after recharging
execution_rate = 10  # Hz
# CODE WAS ADDED HERE
# The LW2 world has 3 charging stations (the old (0, -2) value was from the
# TW10 world). The task FSM uses the left or right one, whichever is closer
# (the center one is behind the central round pillar).
# The chargers are next to the pillars, so with the 0.4 m map inflation the
# center of their rectangles is inside the configuration space and the A*
# would not find a path. The targets below are therefore placed in the free
# part of each charging rectangle (charging works anywhere inside it), as
# verified in the LW1 inflated map.
recharge_targets_wpose = [
    Pose2D(x=-4.8, y=-2.2, theta=-pi/2),  # charger1 (left)
    Pose2D(x=0.0, y=-2.8, theta=-pi/2),   # charger2 (center, behind pillar)
    Pose2D(x=4.8, y=-1.9, theta=-pi/2)]   # charger3 (right)

# CODE WAS ADDED HERE
# ---------------------------------------------------------------------------
# LW2 world locations (values from the assignment and the simulator world)
# ---------------------------------------------------------------------------

# Number of parts to transport (the assignment asks for 3 to 5)
NUM_PARTS_TO_PROCESS = 5

# Part states, as published in the parts_sensor topic
PART_UNPROCESSED = 0
PART_IN_PROCESS = 1
PART_PROCESSED = 2
PART_DELIVERED = 3

# Forklift commands (see MoveForklift action)
FORKLIFT_UP = 1
FORKLIFT_DOWN = 2

# Parts in the input warehouse (all at y = -7)
input_parts_x = [-4.90, -2.45, 0.00, 2.45, 4.90]
INPUT_PARTS_Y = -7.0

# Processing units (all at y = 2)
processing_units_x = [-2.45, 0.00, 2.45]
PROCESSING_UNITS_Y = 2.0

# Delivery units (same row as the processing units, one on each side)
delivery_units_x = [-4.80, 4.80]

# CODE WAS ADDED HERE
# Exit point below the processing/delivery units: after dropping a part
# the robot moves here (straight line out of the unit), so the simulator
# detects that the robot left the unit. The path planning between areas is
# done by the A* planner, so no other intermediate points are needed.
UNITS_EXIT_Y = -0.7

# Approach and drop positions (the part is dropped ~0.4 m in front of the
# robot, and the markers are only detected up to 1 m away, so we approach
# to ~0.7 m to have some margin for the localization error)
INPUT_APPROACH_Y = -6.3  # inside the input stall, ~0.7 m from the part
PROC_DROP_Y = 1.5        # robot position when dropping in a processing unit
PROC_REPICK_Y = 1.3      # robot position to re-detect the processed part
DELIVERY_DROP_Y = 1.5    # robot position when dropping in a delivery unit

# Visual servoing goal tolerances
VS_MAX_DISTANCE = 0.35  # [m]
VS_MAX_BEARING = 0.1   # [rad]

# CODE WAS ADDED HERE
# Backup (reverse) motion after dropping a part, so the forklift does not
# hit the part when the robot rotates to leave the unit
BACKUP_DISTANCE = 0.5  # [m]
BACKUP_LIN_VEL = 0.1   # [m/s]

# CODE WAS ADDED HERE
# Maximum distance to a navigation goal to consider it reached (the A*
# path ends in a grid cell up to ~3.5 cm from the goal and the path
# navigation stops within ~10 cm of the last waypoint, so this needs to
# be larger than the sum of both; it is also below the 20 cm required by
# the charging stations)
NAV_GOAL_TOLERANCE = 0.20  # [m]
