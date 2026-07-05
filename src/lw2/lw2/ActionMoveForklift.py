#!/usr/bin/env python3

# Copyright (c) 2024, Hugo Costelha
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

"""MoveForklift action: Move the forklift to a desired position."""

# Non-ROS modules
import os
from threading import Lock
from enum import Enum
import time

# ROS related modules
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Int8

# Our modules
from ar_utils.action import MoveForklift


# This action name (strip the '.py' preffix)
ACTION_NAME = os.path.basename(__file__)[:-3]


class States(Enum):
    """Enum to represent the forklift states."""

    FORKLIF_UNKNOWN = 0
    FORKLIF_UP = 1
    FORKLIF_DOWN = 2


class MoveForkliftActionServer(Node):
    """Change the forklift position.

    This action accepts a single goal at a time so, requesting a new goal
    while a previous one was alread running, cancels the previous goal.
    """

    def __init__(self):
        """Constructor."""
        super().__init__('action_move_forklift')

        # Create condition to manage access to the goal variable, wich will be
        # accessed in multiple callbacks
        self.goal_handle = None
        self.goal_lock = Lock()
        self.forklift_state = 2

        # Setup publishers for the forklift commands
        self.forklift_pub = self.create_publisher(
            Int8,
            'cmd_forklift', 1)

        # Forklift subscriber (to be used later on)
        self.sub_forklift = None

        # Start the action server.
        # The option ReentrantCallbackGroup allows callbacks to be run in
        # parallel without restrictions
        self.action_server = ActionServer(
            self,
            MoveForklift,
            ACTION_NAME,
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            handle_accepted_callback=self.handle_accepted_cb,
            cancel_callback=self.cancel_cb,
            callback_group=ReentrantCallbackGroup())

    def destroy(self):
        """Destructor."""
        self.action_server.destroy()
        super().destroy_node()

    def goal_cb(self, goal_request):
        """Accept and proccess new goal requested."""
        self.get_logger().info(f'{ACTION_NAME} received new goal request:')
        return GoalResponse.ACCEPT

    def handle_accepted_cb(self, goal_handle):
        """This function runs whenever a new goal is accepted."""
        with self.goal_lock:
            # This server only allows one goal at a time
            if (self.goal_handle is not None) and (self.goal_handle.is_active):
                self.get_logger().info(f'{ACTION_NAME} aborting previous goal')
                # Abort the existing goal
                self.goal_handle.abort()
            self.goal_handle = goal_handle
        # Start runing the execute callback
        goal_handle.execute()

    def cancel_cb(self, goal_handle):
        """Callback that's called when an action cancellation is requested."""
        self.get_logger().info(f'{ACTION_NAME} received a cancel request!')
        # The cancel request was accepted
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        """Callback to execute when the action has a new goal."""
        self.get_logger().info(
            f'Executing action {ACTION_NAME} with position '
            + f'goal {goal_handle.request.position}')

        # Publish desired position (move forklift)
        forklift_pos_cmd = Int8(data=goal_handle.request.position)
        self.forklift_pub.publish(forklift_pos_cmd)

        while rclpy.ok():
            # Wait for new information to arrive
            time.sleep(2.5)
            with self.goal_lock:
                # Only continue if the goal is active and a cancel was not
                # requested
                if (not goal_handle.is_active) or \
                   (goal_handle.is_cancel_requested):
                    if not goal_handle.is_active:
                        self.get_logger().info(f'{ACTION_NAME}: goal aborted')
                    else:  # goal_handle.is_cancel_requested
                        goal_handle.canceled()  # Confirm goal is canceled
                        self.get_logger().info(f'{ACTION_NAME}: goal canceled')
                    return MoveForklift.Result(success=False)
                else:
                    # Return True (success)
                    goal_handle.succeed()
                    return MoveForklift.Result(success=True)


def main(args=None):
    """Main function - start the action server."""
    rclpy.init(args=args)
    forklift_action_server = MoveForkliftActionServer()

    # Use 2 threads to make sure callbacks can run in parallel and the action
    # does not block.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(forklift_action_server)
    executor.spin()


if __name__ == '__main__':
    main()
