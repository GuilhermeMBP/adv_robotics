#!/usr/bin/env python3

# Copyright (c) 2025, Hugo Costelha
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


'''@package docstring
Execute the LW2 task: transport parts from the input warehouse, through a
processing unit, to a delivery unit, using the forklift and visual servoing,
while managing the battery level.
  Use yasmin to implement the FSM: https://github.com/uleroboticsgroup/yasmin
'''

# CODE WAS ADDED HERE (needed to wait for the parts processing)
import time
# CODE WAS ADDED HERE (needed for the rotation states orientations and the
# navigation arrival check)
from math import pi, sqrt

# ROS API
from geometry_msgs.msg import Point
# CODE WAS ADDED HERE (needed for the backup motion after dropping a part
# and for publishing the navigation goals to the A* planner)
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
# CODE WAS ADDED HERE (needed to check if the robot is stopped)
from nav_msgs.msg import Odometry
import rclpy
from sensor_msgs.msg import BatteryState
# CODE WAS ADDED HERE (parts_sensor topic message type)
from std_msgs.msg import UInt8MultiArray

# Yasmin FSM execution
import yasmin
from yasmin import Blackboard
from yasmin import State
from yasmin import StateMachine
from yasmin_ros import ActionState, set_ros_loggers
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import ABORT, CANCEL, SUCCEED
from yasmin_viewer import YasminViewerPub

# Our libraries and functions
from ar_utils import action
import lw2.myglobals as myglobals


class RobotBlackboardSynchronizer:
    """Map between relevant topics and the blackboard."""

    def __init__(self, yasmin_node, blackboard):
        """Store YASMIN relevant objects and subscribe relevant topics."""
        self.node = yasmin_node
        self.blackboard = blackboard
        self.battery_state_subscriber = self.node.create_subscription(
            BatteryState, 'battery/state', self.batteryLevelCallback, 4)
        # CODE WAS ADDED HERE (we also need the parts states in the FSM)
        self.parts_sensor_subscriber = self.node.create_subscription(
            UInt8MultiArray, 'parts_sensor', self.partsSensorCallback, 4)
        # CODE WAS ADDED HERE (the estimated pose and the speed are needed
        # to know when a navigation goal was reached)
        self.pose_subscriber = self.node.create_subscription(
            PoseWithCovarianceStamped, 'pose', self.poseCallback, 1)
        self.odom_subscriber = self.node.create_subscription(
            Odometry, 'odom', self.odomCallback, 1)

    def batteryLevelCallback(self, msg: BatteryState):
        """Store the battery level in the blackboard."""
        self.blackboard['battery_level'] = msg.percentage

    # CODE WAS ADDED HERE
    def partsSensorCallback(self, msg: UInt8MultiArray):
        """Store the parts states in the blackboard."""
        self.blackboard['parts_sensor'] = list(msg.data)

    # CODE WAS ADDED HERE
    def poseCallback(self, msg: PoseWithCovarianceStamped):
        """Store the robot estimated position in the blackboard."""
        self.blackboard['robot_pose'] = (msg.pose.pose.position.x,
                                         msg.pose.pose.position.y)

    # CODE WAS ADDED HERE
    def odomCallback(self, msg: Odometry):
        """Store the robot current speed in the blackboard."""
        self.blackboard['robot_speed'] = \
            abs(msg.twist.twist.linear.x) + abs(msg.twist.twist.angular.z)


# CODE WAS ADDED HERE
class NavigateState(State):
    """Navigates to the position stored in blackboard['nav_goal'], using
    the A* planner and the path navigation node reused from our LW1 work.
    The goal is published in the goal_pose topic, the A* planner computes
    the path in the configuration space and publishes it, and the path
    navigation node follows it. This state waits until the robot is close
    to the goal AND stopped (meaning the path navigation has finished, so
    it no longer sends velocity commands that would conflict with the next
    state). The goal is republished every few seconds, which also replans
    the path from the current robot position."""

    def __init__(self, yasmin_node) -> None:
        # List of possible outcomes
        super().__init__(['arrived'])
        self.node = yasmin_node
        self.goal_pub = yasmin_node.create_publisher(
            PoseStamped, 'goal_pose', 1)

    def execute(self, blackboard: Blackboard) -> str:
        goal_x, goal_y = blackboard['nav_goal']
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.orientation.w = 1.0

        last_pub_time = 0.0
        while True:
            # Wait until we know where the robot is (the localization needs
            # some time to start publishing)
            if blackboard['robot_pose'] is not None:
                robot_x, robot_y = blackboard['robot_pose']
                distance = sqrt((robot_x - goal_x)**2 +
                                (robot_y - goal_y)**2)
                if (distance < myglobals.NAV_GOAL_TOLERANCE) and \
                   (blackboard['robot_speed'] < 0.02):
                    yasmin.YASMIN_LOG_INFO(
                        f'Arrived at ({goal_x:.2f}, {goal_y:.2f}).')
                    return 'arrived'
                # (Re)publish the goal every 5 s. This also covers the case
                # where the first goal was published before the planner had
                # received the map and the pose.
                if time.time() - last_pub_time > 5.0:
                    goal_msg.header.stamp = \
                        self.node.get_clock().now().to_msg()
                    self.goal_pub.publish(goal_msg)
                    last_pub_time = time.time()
            time.sleep(0.5)


# CODE WAS ADDED HERE
class CheckTaskState(State):
    """Decides what to do next: recharge, transport the next part, or finish.
    This state runs at the beginning of each part cycle, so the battery is
    always checked before starting to carry a new part."""

    def __init__(self) -> None:
        # List of possible outcomes
        # CODE WAS ADDED HERE (the part_in_processing outcome was added:
        # since the robot now recharges while the part is being processed,
        # this state must know how to resume a part left at the processing
        # unit instead of always starting at the input warehouse)
        super().__init__(['next_part', 'part_in_processing', 'low_battery',
                          'all_done'])

    def execute(self, blackboard: Blackboard) -> str:
        # If the battery is low, go recharge first
        if blackboard['battery_level'] < myglobals.MIN_POWER_LEVEL:
            # Use the left or right charging station, whichever is on the
            # same side of the map as the robot (the center one is behind
            # the central pillar and harder to reach)
            if blackboard['last_x'] < 0:
                blackboard['charger_idx'] = 0
            else:
                blackboard['charger_idx'] = 2
            charger = \
                myglobals.recharge_targets_wpose[blackboard['charger_idx']]
            # CODE WAS ADDED HERE (navigation goal for the A* planner; the
            # charger index is also stored, so that the fine approach state
            # knows which charging rectangle center to use)
            blackboard['nav_goal'] = (charger.x, charger.y)
            yasmin.YASMIN_LOG_INFO(
                'Battery is low, going to recharge...')
            return 'low_battery'

        # If all parts were delivered, we are done
        # CODE WAS ADDED HERE (this check must run before the check below,
        # otherwise after the last delivery part_idx is already past the
        # end of the parts_sensor list)
        if blackboard['part_idx'] >= myglobals.NUM_PARTS_TO_PROCESS:
            blackboard['message2speak'] = 'All parts were delivered'
            return 'all_done'

        # CODE WAS ADDED HERE
        # If the current part was already left at the processing unit (the
        # robot recharges while the part is being processed), go back to the
        # waiting point outside the processing unit instead of starting a
        # new part cycle. The waiting must be done outside the unit, since
        # entering it while the part is IN_PROCESS resets the processing.
        i = blackboard['part_idx']
        if blackboard['parts_sensor'][i] in (myglobals.PART_IN_PROCESS,
                                             myglobals.PART_PROCESSED):
            proc_x = myglobals.processing_units_x[blackboard['proc_idx']]
            blackboard['nav_goal'] = (proc_x, myglobals.UNITS_EXIT_Y)
            yasmin.YASMIN_LOG_INFO(f'Going back for part {i+1}...')
            return 'part_in_processing'

        # Otherwise, prepare the next part cycle
        i = blackboard['part_idx']
        blackboard['proc_idx'] = i % len(myglobals.processing_units_x)
        # Deliver on the same side of the map as the processing unit. The
        # center processing unit (x = 0) delivers on the right, so that no
        # delivery unit receives more parts than the available lanes (with
        # 5 parts: 2 on the left unit and 3 on the right one).
        if myglobals.processing_units_x[blackboard['proc_idx']] < 0:
            blackboard['delivery_idx'] = 0  # Left delivery unit
        else:
            blackboard['delivery_idx'] = 1  # Right delivery unit
        # CODE WAS ADDED HERE (navigation goal for the A* planner: enter
        # the input warehouse stall, ~0.7 m away from the part)
        blackboard['nav_goal'] = (myglobals.input_parts_x[i],
                                  myglobals.INPUT_APPROACH_Y)
        yasmin.YASMIN_LOG_INFO(f'Going for part {i+1}...')
        return 'next_part'


# CODE WAS ADDED HERE
class WaitProcessingState(State):
    """Waits until the current part is processed (checking the parts_sensor
    information stored in the blackboard). The robot is parked outside the
    processing unit while waiting."""

    def __init__(self) -> None:
        # List of possible outcomes
        super().__init__(['processed'])

    def execute(self, blackboard: Blackboard) -> str:
        i = blackboard['part_idx']
        while blackboard['parts_sensor'][i] != myglobals.PART_PROCESSED:
            time.sleep(1.0)
        yasmin.YASMIN_LOG_INFO(f'Part {i+1} was processed.')
        return 'processed'


# CODE WAS ADDED HERE
class BackupState(State):
    """Moves the robot straight back for a short distance after dropping a
    part, so the forklift does not hit the part when the robot rotates to
    leave the unit. The velocity must be published periodically, because
    the simulator stops the robot if no command arrives for 0.2 s."""

    def __init__(self, yasmin_node) -> None:
        # List of possible outcomes
        super().__init__(['backed_up'])
        self.vel_pub = yasmin_node.create_publisher(Twist, 'cmd_vel', 1)

    def execute(self, blackboard: Blackboard) -> str:
        vel_cmd = Twist()
        duration = myglobals.BACKUP_DISTANCE / myglobals.BACKUP_LIN_VEL
        start_time = time.time()
        while time.time() - start_time < duration:
            vel_cmd.linear.x = -myglobals.BACKUP_LIN_VEL
            self.vel_pub.publish(vel_cmd)
            time.sleep(0.1)
        # Stop the robot
        vel_cmd.linear.x = 0.0
        self.vel_pub.publish(vel_cmd)
        return 'backed_up'


class FailureState(State):
    """Represents the failure state."""

    def __init__(self) -> None:
        # List of possible outcomes
        super().__init__(['failure'])

    def execute(self, blackboard: Blackboard) -> str:
        # We are simply returning a fixed outcome, but the idea is to run
        # the code corresponding to this state and, depending how thing run,
        # return the corresponding outcome
        return 'failure'


def main(args=None):
    """
    @brief  Main function for the LW2 task using YASMIN.

    Main function - runs the LW2 parts transportation task, implemented
    as a FSM using YASMIN.
    """
    rclpy.init(args=args)

    # Set ROS 2 logs
    set_ros_loggers()

    # Get YASMIN node instance
    yasmin_node = YasminNode.get_instance()

    # Create a finite state machine (FSM)
    # CODE WAS ADDED HERE (added the success outcome for when the task ends)
    sm = StateMachine(outcomes=["failure", "success"])

    # Create the blackboard
    blackboard = Blackboard()
    # Initialize the message to be spoken as an empty string
    blackboard['message2speak'] = ''
    # Initialize the stored battery level to 1.0 (100%)
    blackboard['battery_level'] = 1.0
    # CODE WAS ADDED HERE (task related information)
    # Last known state of each part (from the parts_sensor topic)
    blackboard['parts_sensor'] = \
        [myglobals.PART_UNPROCESSED] * myglobals.NUM_PARTS_TO_PROCESS
    blackboard['part_idx'] = 0  # Part currently being transported
    blackboard['proc_idx'] = 0  # Processing unit for the current part
    blackboard['delivery_idx'] = 0  # Delivery unit for the current part
    blackboard['nav_goal'] = None  # Target position for the A* navigation
    blackboard['robot_pose'] = None  # Robot estimated position (x, y)
    blackboard['robot_speed'] = 0.0  # Robot current speed
    blackboard['last_x'] = 0.0  # Last known robot x position
    blackboard['charger_idx'] = 0  # Charging station to use when recharging
    # Number of parts already delivered in each delivery unit, used to
    # choose a free lane for the next part
    blackboard['delivered_count'] = [0, 0]

    #
    # Add each state to the FSM
    #

    # CODE WAS ADDED HERE (all the states below implement the LW2 task)

    #############################################################
    # CheckTask state: decide between recharging, going for the next part,
    # or finishing the task
    sm.add_state(
        'CHECK_TASK',
        CheckTaskState(),
        transitions={
            'next_part': 'NAV_TO_INPUT',
            'part_in_processing': 'NAV_BACK_TO_PROC',
            'low_battery': 'NAV_TO_CHARGER',
            'all_done': 'SPEAK_DONE',
        },
    )

    # CODE WAS ADDED HERE
    #############################################################
    # NavToInput state: navigate to the part in the input warehouse, using
    # the A* planner and the path navigation (reused from the LW1)
    sm.add_state(
        'NAV_TO_INPUT',
        NavigateState(yasmin_node),
        transitions={
            'arrived': 'ROTATE_TO_PART',
        },
    )

    #############################################################
    # RotateToPart state: face the part before the visual servoing, since
    # Move2Pos does not control the final orientation and the markers
    # camera has a limited field of view. The parts in the input warehouse
    # are to the south of the robot (-90 degrees).
    def create_goal_cb_rotate_to_part(blackboard: Blackboard):
        # Create the desired goal
        goal = action.Rotate2Angle.Goal(target_orientation=-pi/2)
        return goal

    def result_handler_cb_rotate_to_part(blackboard: Blackboard,
                                         response: action.Rotate2Angle.Result):
        return SUCCEED
    sm.add_state(
        'ROTATE_TO_PART',
        ActionState(
            action_type=action.Rotate2Angle,
            action_name='ActionRotate2Angle',
            create_goal_handler=create_goal_cb_rotate_to_part,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=result_handler_cb_rotate_to_part
        ),
        transitions={
            SUCCEED: 'APPROACH_PART',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    #############################################################
    # ApproachPart state: fine approach to the part using visual servoing
    def create_goal_cb_approach_part(blackboard: Blackboard):
        # Create the desired goal
        goal = action.MoveVisualServoing.Goal(
            max_distance=myglobals.VS_MAX_DISTANCE,
            max_bearing=myglobals.VS_MAX_BEARING)
        return goal

    def result_handler_cb_approach_part(
            blackboard: Blackboard,
            response: action.MoveVisualServoing.Result):
        # Print the result
        yasmin.YASMIN_LOG_INFO('Action MoveVisualServoing finished with ' +
                               'success: ' + str(response.success))
        return SUCCEED
    sm.add_state(
        'APPROACH_PART',
        ActionState(
            action_type=action.MoveVisualServoing,
            action_name='ActionMoveVisualServoing',
            create_goal_handler=create_goal_cb_approach_part,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=result_handler_cb_approach_part
        ),
        transitions={
            SUCCEED: 'LIFT_PART',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    #############################################################
    # LiftPart state: raise the forklift with the part, and prepare the
    # route to the processing unit
    def create_goal_cb_lift_part(blackboard: Blackboard):
        # Create the desired goal
        goal = action.MoveForklift.Goal(position=myglobals.FORKLIFT_UP)
        return goal

    def result_handler_cb_lift_part(blackboard: Blackboard,
                                    response: action.MoveForklift.Result):
        # CODE WAS ADDED HERE
        # The next navigation goal is the processing unit drop position
        proc_x = myglobals.processing_units_x[blackboard['proc_idx']]
        blackboard['nav_goal'] = (proc_x, myglobals.PROC_DROP_Y)
        return SUCCEED
    sm.add_state(
        'LIFT_PART',
        ActionState(
            action_type=action.MoveForklift,
            action_name='ActionMoveForklift',
            create_goal_handler=create_goal_cb_lift_part,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=result_handler_cb_lift_part
        ),
        transitions={
            SUCCEED: 'NAV_TO_PROCESSING',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    # CODE WAS ADDED HERE
    #############################################################
    # NavToProcessing state: navigate to the processing unit
    sm.add_state(
        'NAV_TO_PROCESSING',
        NavigateState(yasmin_node),
        transitions={
            'arrived': 'DROP_PART',
        },
    )

    #############################################################
    # DropPart state: lower the forklift, leaving the part in the
    # processing unit
    def create_goal_cb_drop_part(blackboard: Blackboard):
        # Create the desired goal
        goal = action.MoveForklift.Goal(position=myglobals.FORKLIFT_DOWN)
        return goal

    def result_handler_cb_drop_part(blackboard: Blackboard,
                                    response: action.MoveForklift.Result):
        return SUCCEED
    sm.add_state(
        'DROP_PART',
        ActionState(
            action_type=action.MoveForklift,
            action_name='ActionMoveForklift',
            create_goal_handler=create_goal_cb_drop_part,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=result_handler_cb_drop_part
        ),
        transitions={
            SUCCEED: 'BACKUP_PROC',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    # CODE WAS ADDED HERE
    #############################################################
    # BackupProc state: move back a little after dropping the part, so the
    # forklift does not hit it when the robot rotates to leave
    sm.add_state(
        'BACKUP_PROC',
        BackupState(yasmin_node),
        transitions={
            'backed_up': 'EXIT_PROCESSING',
        },
    )

    #############################################################
    # ExitProcessing state: leave the processing unit, since the part only
    # starts being processed when the robot is outside the unit
    def create_goal_cb_exit_processing(blackboard: Blackboard):
        # Create the desired goal
        proc_x = myglobals.processing_units_x[blackboard['proc_idx']]
        goal = action.Move2Pos.Goal(target_position=Point(
            x=proc_x, y=myglobals.UNITS_EXIT_Y, z=0.0))
        return goal

    def result_handler_cb_exit_processing(blackboard: Blackboard,
                                          response: action.Move2Pos.Result):
        # CODE WAS ADDED HERE
        # Instead of waiting parked outside the unit, the robot always goes
        # to recharge while the part is being processed (20 to 50 s), using
        # the charger on the same side as the processing unit. This uses the
        # processing dead time to keep the battery topped up, so there is no
        # need to monitor the battery level while waiting.
        if myglobals.processing_units_x[blackboard['proc_idx']] <= 0:
            blackboard['charger_idx'] = 0
        else:
            blackboard['charger_idx'] = 2
        charger = \
            myglobals.recharge_targets_wpose[blackboard['charger_idx']]
        blackboard['nav_goal'] = (charger.x, charger.y)
        return SUCCEED
    sm.add_state(
        'EXIT_PROCESSING',
        ActionState(
            action_type=action.Move2Pos,
            action_name='ActionMove2Pos',
            create_goal_handler=create_goal_cb_exit_processing,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=result_handler_cb_exit_processing
        ),
        transitions={
            SUCCEED: 'NAV_TO_CHARGER',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    # CODE WAS ADDED HERE
    #############################################################
    # NavBackToProc state: after recharging, navigate back to the waiting
    # point outside the processing unit (the robot must stay outside the
    # unit until the part is processed)
    sm.add_state(
        'NAV_BACK_TO_PROC',
        NavigateState(yasmin_node),
        transitions={
            'arrived': 'WAIT_PROCESSING',
        },
    )

    #############################################################
    # WaitProcessing state: wait until the part is processed (usually it
    # already is, since the robot recharged in the meantime)
    sm.add_state(
        'WAIT_PROCESSING',
        WaitProcessingState(),
        transitions={
            'processed': 'REENTER_PROCESSING',
        },
    )

    #############################################################
    # ReenterProcessing state: get close to the processed part again (about
    # 1 m away, so the markers camera is able to detect it)
    def create_goal_cb_reenter_processing(blackboard: Blackboard):
        # Create the desired goal
        proc_x = myglobals.processing_units_x[blackboard['proc_idx']]
        goal = action.Move2Pos.Goal(target_position=Point(
            x=proc_x, y=myglobals.PROC_REPICK_Y, z=0.0))
        return goal

    def result_handler_cb_reenter_processing(blackboard: Blackboard,
                                             response: action.Move2Pos.Result):
        return SUCCEED
    sm.add_state(
        'REENTER_PROCESSING',
        ActionState(
            action_type=action.Move2Pos,
            action_name='ActionMove2Pos',
            create_goal_handler=create_goal_cb_reenter_processing,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=result_handler_cb_reenter_processing
        ),
        transitions={
            SUCCEED: 'ROTATE_TO_PART2',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    #############################################################
    # RotateToPart2 state: face the processed part before the visual
    # servoing. The part in the processing unit is to the north of the
    # robot (90 degrees).
    def create_goal_cb_rotate_to_part2(blackboard: Blackboard):
        # Create the desired goal
        goal = action.Rotate2Angle.Goal(target_orientation=pi/2)
        return goal

    def result_handler_cb_rotate_to_part2(
            blackboard: Blackboard,
            response: action.Rotate2Angle.Result):
        return SUCCEED
    sm.add_state(
        'ROTATE_TO_PART2',
        ActionState(
            action_type=action.Rotate2Angle,
            action_name='ActionRotate2Angle',
            create_goal_handler=create_goal_cb_rotate_to_part2,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=result_handler_cb_rotate_to_part2
        ),
        transitions={
            SUCCEED: 'REAPPROACH_PART',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    #############################################################
    # ReapproachPart state: fine approach to the processed part using
    # visual servoing (same goal as the ApproachPart state)
    sm.add_state(
        'REAPPROACH_PART',
        ActionState(
            action_type=action.MoveVisualServoing,
            action_name='ActionMoveVisualServoing',
            create_goal_handler=create_goal_cb_approach_part,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=result_handler_cb_approach_part
        ),
        transitions={
            SUCCEED: 'LIFT_PART2',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    #############################################################
    # LiftPart2 state: raise the forklift with the processed part, and
    # prepare the route to the delivery unit
    def result_handler_cb_lift_part2(blackboard: Blackboard,
                                     response: action.MoveForklift.Result):
        # CODE WAS ADDED HERE
        # The next navigation goal is the delivery unit drop position, using
        # a free lane: the parts already delivered stay inside the unit, so
        # each new part is dropped 0.7 m to the side of the previous one to
        # avoid crashing the carried part into them
        count = blackboard['delivered_count'][blackboard['delivery_idx']]
        delivery_x = \
            myglobals.delivery_units_x[blackboard['delivery_idx']] + \
            myglobals.DELIVERY_LANE_OFFSETS[count]
        blackboard['nav_goal'] = (delivery_x, myglobals.DELIVERY_DROP_Y)
        return SUCCEED
    sm.add_state(
        'LIFT_PART2',
        ActionState(
            action_type=action.MoveForklift,
            action_name='ActionMoveForklift',
            create_goal_handler=create_goal_cb_lift_part,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=result_handler_cb_lift_part2
        ),
        transitions={
            SUCCEED: 'NAV_TO_DELIVERY',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    # CODE WAS ADDED HERE
    #############################################################
    # NavToDelivery state: navigate to the delivery unit
    sm.add_state(
        'NAV_TO_DELIVERY',
        NavigateState(yasmin_node),
        transitions={
            'arrived': 'DROP_DELIVERY',
        },
    )

    #############################################################
    # DropDelivery state: lower the forklift, leaving the part in the
    # delivery unit
    sm.add_state(
        'DROP_DELIVERY',
        ActionState(
            action_type=action.MoveForklift,
            action_name='ActionMoveForklift',
            create_goal_handler=create_goal_cb_drop_part,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=result_handler_cb_drop_part
        ),
        transitions={
            SUCCEED: 'BACKUP_DELIVERY',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    # CODE WAS ADDED HERE
    #############################################################
    # BackupDelivery state: same as BackupProc, but in the delivery unit
    sm.add_state(
        'BACKUP_DELIVERY',
        BackupState(yasmin_node),
        transitions={
            'backed_up': 'EXIT_DELIVERY',
        },
    )

    #############################################################
    # ExitDelivery state: leave the delivery unit (the part is only marked
    # as delivered when the robot is outside the unit), and move on to the
    # next part
    def create_goal_cb_exit_delivery(blackboard: Blackboard):
        # Create the desired goal
        delivery_x = myglobals.delivery_units_x[blackboard['delivery_idx']]
        goal = action.Move2Pos.Goal(target_position=Point(
            x=delivery_x, y=myglobals.UNITS_EXIT_Y, z=0.0))
        return goal

    def result_handler_cb_exit_delivery(blackboard: Blackboard,
                                        response: action.Move2Pos.Result):
        # Print the result
        yasmin.YASMIN_LOG_INFO(
            f'Part {blackboard["part_idx"]+1} was delivered!')
        # Store the robot position and proceed to the next part
        blackboard['last_x'] = \
            myglobals.delivery_units_x[blackboard['delivery_idx']]
        # CODE WAS ADDED HERE (one more part in this delivery unit, so the
        # next one delivered here will use the next free lane)
        blackboard['delivered_count'][blackboard['delivery_idx']] += 1
        blackboard['part_idx'] += 1
        # CODE WAS ADDED HERE
        # After a successful delivery the robot always goes to recharge
        # (using the charger on the same side as the delivery unit), before
        # starting the next part. Recharging only while the part was being
        # processed was not enough: the trip from the delivery unit to the
        # input warehouse and back to a processing unit could take longer
        # than the battery allowed, ending with the battery at 0. Charging
        # at these two fixed moments guarantees the battery never runs out.
        if myglobals.delivery_units_x[blackboard['delivery_idx']] < 0:
            blackboard['charger_idx'] = 0
        else:
            blackboard['charger_idx'] = 2
        charger = \
            myglobals.recharge_targets_wpose[blackboard['charger_idx']]
        blackboard['nav_goal'] = (charger.x, charger.y)
        return SUCCEED
    sm.add_state(
        'EXIT_DELIVERY',
        ActionState(
            action_type=action.Move2Pos,
            action_name='ActionMove2Pos',
            create_goal_handler=create_goal_cb_exit_delivery,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=result_handler_cb_exit_delivery
        ),
        transitions={
            # CODE WAS ADDED HERE (recharge after each delivery; the
            # CHECK_TASK state then decides the next part when the
            # recharging sequence ends)
            SUCCEED: 'NAV_TO_CHARGER',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    # CODE WAS ADDED HERE
    #############################################################
    # NavToCharger state: navigate to the charging station
    sm.add_state(
        'NAV_TO_CHARGER',
        NavigateState(yasmin_node),
        transitions={
            'arrived': 'MOVE_TO_CHARGER_CENTER',
        },
    )

    # CODE WAS ADDED HERE
    #############################################################
    # MoveToChargerCenter state: fine approach to the center of the charging
    # rectangle, entering sideways. The A* navigation target is ~0.8 m to
    # the side of the charger (the center is inside the inflated map, and
    # the 0.2 m navigation tolerance could leave the robot outside the
    # 0.5x0.5 m charging rectangle, in which case the simulator refuses to
    # charge). Move2Pos does not use the map, so it can do the last few
    # centimeters to the exact center; and since the robot comes from the
    # side, the forks stay parallel to the pillar to the south of the
    # charger instead of hitting it.
    def create_goal_cb_move_to_charger(blackboard: Blackboard):
        # Create the desired goal
        charger = myglobals.recharge_centers_wpose[blackboard['charger_idx']]
        goal = action.Move2Pos.Goal(target_position=Point(
            x=charger.x, y=charger.y, z=0.0))
        return goal

    def result_handler_cb_move_to_charger(blackboard: Blackboard,
                                          response: action.Move2Pos.Result):
        return SUCCEED
    sm.add_state(
        'MOVE_TO_CHARGER_CENTER',
        ActionState(
            action_type=action.Move2Pos,
            action_name='ActionMove2Pos',
            create_goal_handler=create_goal_cb_move_to_charger,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=result_handler_cb_move_to_charger
        ),
        transitions={
            SUCCEED: 'STOP_AT_CHARGER',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    #############################################################
    # StopAtCharger state: make sure the robot is fully stopped, since the
    # simulator only starts charging if the robot is stopped
    def create_goal_cb_stop(blackboard: Blackboard):
        # Create the desired goal
        goal = action.Stop.Goal()
        return goal

    def result_handler_cb_stop(blackboard: Blackboard,
                               response: action.Stop.Result):
        # Print the result
        yasmin.YASMIN_LOG_INFO('Action Stop finished with robot stopped: ' +
                               str(response.is_stopped))
        return SUCCEED
    sm.add_state(
        'STOP_AT_CHARGER',
        ActionState(
            action_type=action.Stop,
            action_name='ActionStop',
            create_goal_handler=create_goal_cb_stop,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=result_handler_cb_stop
        ),
        transitions={
            SUCCEED: 'RECHARGE',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    #############################################################
    # Recharge state
    def create_goal_cb_recharge(blackboard: Blackboard):
        # Create the desired goal
        goal = action.Recharge.Goal(
            target_battery_level=myglobals.MAX_POWER_LEVEL)
        return goal

    def result_handler_cb_recharge(blackboard: Blackboard,
                                   response: action.Recharge.Result):
        # Print the result
        yasmin.YASMIN_LOG_INFO('Action Recharge finished with battery at: ' +
                               f'{response.battery_level*100:.0f}%')
        return SUCCEED
    sm.add_state(
        'RECHARGE',
        ActionState(
            action_type=action.Recharge,
            action_name='ActionRecharge',
            create_goal_handler=create_goal_cb_recharge,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=result_handler_cb_recharge
        ),
        transitions={
            SUCCEED: 'EXIT_CHARGER',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    # CODE WAS ADDED HERE
    #############################################################
    # ExitCharger state: leave the charger moving forward to a point to the
    # north of the charger center. The robot may end the entering motion
    # with an arbitrary orientation (Move2Pos does not control the final
    # orientation), so a blind backup could push it against the pillar and
    # into the inflated area, where the A* has no solution. Moving to a
    # point to the north is always safe: the rotation towards north never
    # sweeps the forks through the pillar (which is to the south), and the
    # exit point is in free configuration space.
    def create_goal_cb_exit_charger(blackboard: Blackboard):
        # Create the desired goal
        charger = myglobals.recharge_centers_wpose[blackboard['charger_idx']]
        goal = action.Move2Pos.Goal(target_position=Point(
            x=charger.x,
            y=charger.y + myglobals.CHARGER_EXIT_OFFSET,
            z=0.0))
        return goal

    def result_handler_cb_exit_charger(blackboard: Blackboard,
                                       response: action.Move2Pos.Result):
        return SUCCEED
    sm.add_state(
        'EXIT_CHARGER',
        ActionState(
            action_type=action.Move2Pos,
            action_name='ActionMove2Pos',
            create_goal_handler=create_goal_cb_exit_charger,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=result_handler_cb_exit_charger
        ),
        transitions={
            SUCCEED: 'CHECK_TASK',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    #############################################################
    # SpeakDone state: announce the end of the task
    def create_goal_cb_speak(blackboard: Blackboard):
        # Create the desired goal
        goal = action.SpeakText.Goal(text_to_speak=blackboard['message2speak'])
        return goal

    def result_handler_cb_speak(blackboard: Blackboard,
                                response: action.SpeakText.Result):
        # Print the result
        yasmin.YASMIN_LOG_INFO('Action SpeakText succeeded: ' +
                               str(response.text_spoken))
        return SUCCEED
    sm.add_state(
        'SPEAK_DONE',
        ActionState(
            action_type=action.SpeakText,
            action_name='ActionSpeakText',
            create_goal_handler=create_goal_cb_speak,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=result_handler_cb_speak
        ),
        transitions={
            SUCCEED: 'success',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    #############################################################
    # Failure state
    sm.add_state(
        'FAILURE',
        FailureState(),
        transitions={
            'failure': 'failure'
        },
    )

    # Publish FSM information for visualization
    # CODE WAS ADDED HERE (in YASMIN 5 the FSM is the first argument; with
    # the name first, the viewer tried to validate the name string and
    # failed with "'str' object has no attribute 'validate'")
    YasminViewerPub(sm, 'LW2_YASMIN')

    #############################################################
    # Get our robot state to backboard synchronizer
    RobotBlackboardSynchronizer(yasmin_node, blackboard)

    #############################################################
    # Execute the FSM
    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        rclpy.shutdown()

    print('Done...')


if __name__ == '__main__':
    main()
