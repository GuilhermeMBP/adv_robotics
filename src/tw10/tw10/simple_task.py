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


"""Simple task using a FSM implemented in plain Python.

Execute a simple task, formed by the following sequential execution:
  - Action Move2Pos to position (0, -2) [m]
  - Action Rotate2Angle to orientation -90 deg
  - Action Stop
  - Action Recharge to 90%
"""

# Our libraries and functions
import ar_utils.action as action
import tw10.myglobals as myglobals

# ROS API
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point
from action_msgs.msg import GoalStatus

# Other libraries
from numpy import degrees
from enum import Enum


class States(Enum):
    """Enumeration of possible FSM states."""

    STATE_MOTION = 'STATE_MOTION'
    STATE_ROTATION = 'STATE_ROTATION'
    STATE_STOP = 'STATE_STOP'
    STATE_RECHARGING = 'STATE_RECHARGING'
    STATE_FAILURE = 'STATE_FAILURE'
    STATE_SUCCESS = 'STATE_SUCCESS'


class SimpleTask(Node):
    """ROS node for a simple sequential task as a finite state machine.

    The task executes a motion, rotation, stop, and recharge sequence.
    """

    def __init__(self) -> None:
        """Initialize the node and set the initial state."""
        super().__init__('tw10_simple_task')
        # Set the initial state
        self.state = States.STATE_MOTION
        # Build internal states structure for calling the correspond
        # methods
        self.state_call = {
            'STATE_MOTION': self.STATE_MOTION,
            'STATE_ROTATION': self.STATE_ROTATION,
            'STATE_STOP': self.STATE_STOP,
            'STATE_RECHARGING': self.STATE_RECHARGING,
            'STATE_FAILURE': self.STATE_FAILURE,
            'STATE_SUCCESS': self.STATE_SUCCESS
        }

    def create_action_clients(self):
        """Create all the action clients.

        It will wait for each action server
        to be available.
        """
        # Create the client for each action
        self.client_move2pos = ActionClient(
            self, action.Move2Pos, 'ActionMove2Pos')
        self.client_speak_text = ActionClient(
            self, action.SpeakText, 'ActionSpeakText')
        self.client_rotate2angle = ActionClient(
            self, action.Rotate2Angle, 'ActionRotate2Angle')
        self.client_recharge = ActionClient(
            self, action.Recharge, 'ActionRecharge')
        self.client_stop = ActionClient(
            self, action.Stop, 'ActionStop')
        # Wait for each action server to be up and running
        self.client_move2pos.wait_for_server()
        self.client_speak_text.wait_for_server()
        self.client_rotate2angle.wait_for_server()
        self.client_recharge.wait_for_server()
        self.client_stop.wait_for_server()

    #
    # State methods
    #

    def STATE_MOTION(self):
        """Execute the motion state: move to a target position."""
        self.get_logger().info(f'Running state {self.state.value}...')

        # Create the goal position and send it to the action server.
        # We will wait for the action to finish and return a result.
        goal = action.Move2Pos.Goal(target_position=Point(
            x=myglobals.recharge_targets_wpose.x,
            y=myglobals.recharge_targets_wpose.y,
            z=0.0))
        result_response = self.client_move2pos.send_goal(goal)
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn('Action Move2Pos failed with result:'
                                   + str(result_response.result.final_pose))
            # Select next state
            self.state = States.STATE_FAILURE
        else:
            self.get_logger().info('Action Move2Pos suceeded with result:'
                                   + str(result_response.result.final_pose))
            # Select next state
            self.state = States.STATE_ROTATION

    def STATE_ROTATION(self):
        """Execute the rotation state: rotate to a target angle."""
        self.get_logger().info(f'Running state {self.state.value}...')

        # Create the goal angle and send it to the action server.
        # We will wait for the action to finish and return a result.
        goal = action.Rotate2Angle.Goal(
            target_orientation=myglobals.recharge_targets_wpose.theta)
        result_response = self.client_rotate2angle.send_goal(goal)
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                'Action Rotate2Angle failed with result:'
                + f'{degrees(result_response.result.final_orientation):.2f}')
            # Select next state
            self.state = States.STATE_FAILURE
        else:
            self.get_logger().info(
                'Action Rotate2Angle suceeded with result:'
                + f'{degrees(result_response.result.final_orientation):.2f}')
            # Select next state
            self.state = States.STATE_STOP

    def STATE_STOP(self):
        """Execute the stop state: stop the robot."""
        self.get_logger().info(f'Running state {self.state.value}...')

        # Create the goal stop (empty) send it to the action server.
        # We will wait for the action to finish and return a result.
        goal = action.Stop.Goal()  # Nothing specific
        result_response = self.client_stop.send_goal(goal)
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Action Stop failed with result:'
                                   + str(result_response.result.is_stopped))
            # Select next state
            self.state = States.STATE_FAILURE
        else:
            self.get_logger().info('Action Stop suceeded with result:'
                                   + str(result_response.result.is_stopped))
            # Select next state
            self.state = States.STATE_RECHARGING

    def STATE_RECHARGING(self):
        """Execute the recharging state: recharge the battery."""
        self.get_logger().info(f'Running state {self.state.value}...')

        # Create the goal battery level and send it to the action server.
        # We will wait for the action to finish and return a result.
        goal = action.Recharge.Goal(target_battery_level=1.0)
        result_response = self.client_recharge.send_goal(goal)
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                'Action Recharge failed with result:'
                + str(result_response.result.battery_level))
            # Select next state
            self.state = States.STATE_FAILURE
        else:
            self.get_logger().info(
                'Action Recharge suceeded with result:'
                + str(result_response.result.battery_level))

            # In this case, run the speak text action
            goal = action.SpeakText.Goal(
                text_to_speak='Battery is now fully charged!')
            result_response = self.client_speak_text.send_goal(goal)
            if result_response.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Action SpeakText failed!')
            else:
                self.get_logger().info('Action SpeakText suceeded!')

            # Select next state
            self.state = States.STATE_SUCCESS

    def STATE_FAILURE(self):
        """Execute the failure state: log the failure and shutdown."""
        self.get_logger().info(f'Running state {self.state}...')

        # In this case, shutdown
        rclpy.shutdown()

    def STATE_SUCCESS(self):
        """Execute the success state: log the success and shutdown."""
        self.get_logger().info(f'Running state {self.state}...')

        # In this case, shutdown
        rclpy.shutdown()

    def run_task(self):
        """Execute a sequence of pre-defined actions."""
        while rclpy.ok():
            # Execute the current state
            self.state_call[self.state.value]()

        self.get_logger().info('Done...')


def main(args=None):
    """Main function - runs a simple sequential task with few actions."""
    rclpy.init(args=args)

    # Initializes a node so that the action client can publish and subscribe
    # over ROS.
    simple_task = SimpleTask()

    # Create the actions clients (wait for the action servers to be available)
    simple_task.create_action_clients()

    # We will use several threads to make sure that the callbacks are spinned
    # while we are waiting for the goal requests to finish, i.e., while we wait
    # for the action servers responses
    executor = MultiThreadedExecutor(num_threads=2)
    # Include our Node callbacks
    executor.add_node(simple_task)
    # Include our main task function
    executor.create_task(simple_task.run_task)
    # Spin both the Node and the function runTask
    executor.spin()

    print('Done...')


if __name__ == '__main__':
    main()
