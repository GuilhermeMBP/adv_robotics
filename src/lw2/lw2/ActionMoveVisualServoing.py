#!/usr/bin/env python3

# Copyright (c) 2024, Hugo Costelha
# All rights reserved
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
MoveVisualServoing action: move to the closest marker until the marker relative
position and bearing are smaller than a the desired values.
'''

# Non-ROS modules
import os
from math import radians  # CODE WAS ADDED HERE
from threading import Lock, Event
import functools

# ROS related modules
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
# CODE WAS ADDED HERE (needed for the blind approach deadline)
from rclpy.duration import Duration

# Our modules
from ar_utils.action import MoveVisualServoing
from ar_py_utils.utils import clipValue  # CODE WAS ADDED HERE
from geometry_msgs.msg import Twist  # CODE WAS ADDED HERE
from markers_msgs.msg import Markers

# This action name (strip the '.py' preffix)
ACTION_NAME = os.path.basename(__file__)[:-3]


class MoveVisualServoingActionServer(Node):
    '''
        Given a maximum distance and bearing, move the robot until its relative
        position to the nearest marker is smaller that the desired distance and
        bearing.
    '''
    def __init__(self):
        super().__init__('action_move_visual_servoing')

        # Create condition to manage access to the goal variable, wich will be
        # accessed in multiple callbacks
        self.goal_handle = None
        self.goal_lock = Lock()

        ''' Initialize members for navigation control '''
        self.curr_detected_markers = None

        # Robot navigation/motion related constants and variables
        self.MAX_LIN_VEL = 1.0  # Maximum linear speed [m/s]
        self.MAX_ANG_VEL = 1.57  # Maximu angular speed (90°/s) [rad/s]

        # CODE WAS ADDED HERE
        # Visual servoing control parameters (proportional controller gains
        # and the maximum bearing above which the robot rotates in place)
        self.Kp_lin_vel = 0.5
        self.Kp_ang_vel = 1.5
        self.max_angle_to_target = radians(30.0)
        # Blind final approach (markers not visible below this range)
        self.blind_zone_range = 0.70   
        self.blind_lin_vel = 0.15      

        # CODE WAS ADDED HERE
        # Velocity publisher and reusable Twist message (the original file
        # used self.vel_pub without ever creating it)
        self.vel_cmd = Twist()
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)

        # Markers subscriber (to be used later on)
        self.sub_markers = None

        # Start the actual action server
        self.action_server = ActionServer(
            self,
            MoveVisualServoing,
            ACTION_NAME,
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            handle_accepted_callback=self.handle_accepted_cb,
            cancel_callback=self.cancel_cb,
            callback_group=ReentrantCallbackGroup())

    def destroy(self):
        ''' Destructor '''
        self.action_server.destroy()
        super().destroy_node()

    def goal_cb(self, goal_request):
        '''This function is called when a new goal is requested. Currently it
        always accept a new goal.'''
        self.get_logger().info(f'{ACTION_NAME} received new goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_cb(self, goal_handle):
        ''' This function runs whenever a new goal is accepted.'''
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
        ''' Callback that's called when an action cancellation is requested '''
        self.get_logger().info(f'{ACTION_NAME} received a cancel request!')
        # The cancel request was accepted
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        ''' Callback to execute when the action has a new goal '''
        self.get_logger().info(
            f'Executing action {ACTION_NAME} with maximum distance ' +
            f'[{goal_handle.request.max_bearing:0.2f}' +
            ', and maximum bearing' +
            f' {goal_handle.request.max_distance:0.2f}]. [m]')

        # CODE WAS ADDED HERE
        # Reset detected markers so we don't reuse stale data from a
        # previous goal
        self.curr_detected_markers = None

        # Wait for a confimation (trigger), either due to the goal having
        # succeeded, or the goal having been cancelled.
        trigger_event = Event()  # Flag is intially set to False

        # Setup subscriber for the markers
        with self.goal_lock:
            if self.sub_markers is None:
                self.sub_markers = self.create_subscription(
                    Markers,
                    'markers',
                    functools.partial(self.markersCallback,
                                      goal_handle=goal_handle,
                                      trigger_event=trigger_event),
                    1,
                    callback_group=ReentrantCallbackGroup())

        # Used for feedback purposes
        feedback = MoveVisualServoing.Feedback()

        last_range = None
        last_bearing = None
        # CODE WAS ADDED HERE
        # Deadline for the blind final approach (None while not blind).
        # We cannot simply sleep while moving, because the simulator stops
        # the robot if no velocity command is received for 0.2 s, so we keep
        # publishing in the loop until this deadline is reached.
        blind_end_time = None

        while rclpy.ok():
            # Wait for new information to arrive
            if trigger_event.wait(5.0) is False:
                self.get_logger().warn(f'{ACTION_NAME} is still running')
                with self.goal_lock:
                    if self.sub_markers is None:
                        self.sub_markers = self.create_subscription(
                            Markers,
                            'markers',
                            functools.partial(self.markersCallback,
                                              goal_handle=goal_handle,
                                              trigger_event=trigger_event),
                            1,
                            callback_group=ReentrantCallbackGroup())
            else:
                # If the event was triggered, clear it
                trigger_event.clear()

            with self.goal_lock:
                # Check if the goal is no longer active or if a cancel was
                # requested.
                if (not goal_handle.is_active) or \
                   (goal_handle.is_cancel_requested):
                    if not goal_handle.is_active:
                        self.get_logger().info(f'{ACTION_NAME}: goal aborted')
                    else:  # goal_handle.is_cancel_requested
                        goal_handle.canceled()  # Confirm goal is canceled
                        self.get_logger().info(f'{ACTION_NAME}: goal canceled')
                    # No need for the callback anymore
                    if self.sub_markers is not None:
                        self.destroy_subscription(self.sub_markers)
                        self.sub_markers = None
                    # Return whatever result we have so far
                    return MoveVisualServoing.Result(success=False)

                ''' Control the robot velocity to reach the desired goal '''

                # CODE WAS ADDED HERE
                # =====================================================
                # STEP 1: Determine the marker to approach
                # Strategy: choose the closest marker (smallest range).
                # If no markers are visible, stop and abort (lost target).
                # =====================================================
                # CODE WAS ADDED HERE
                # If we are in the blind final approach, keep sending the
                # forward velocity on every loop iteration (the markers
                # messages keep arriving with 0 markers, which keeps this
                # loop running). When the deadline is reached, we stop and
                # finish with success.
                if blind_end_time is not None:
                    if self.get_clock().now() < blind_end_time:
                        self.vel_cmd.linear.x = self.blind_lin_vel
                        self.vel_cmd.angular.z = 0.0
                        self.vel_pub.publish(self.vel_cmd)
                        continue  # Keep going until the deadline
                    # Deadline reached: stop the robot and succeed
                    self.vel_cmd.linear.x = 0.0
                    self.vel_cmd.angular.z = 0.0
                    self.vel_pub.publish(self.vel_cmd)
                    if self.sub_markers is not None:
                        self.destroy_subscription(self.sub_markers)
                        self.sub_markers = None
                    goal_handle.succeed()
                    self.get_logger().info(
                        f'{ACTION_NAME} succeeded (blind phase)!')
                    return MoveVisualServoing.Result(success=True)

                markers = self.curr_detected_markers
                if (markers is None) or (markers.num_markers == 0):
                    # Lost close and aligned? Expected: finish blindly.
                    if (last_range is not None) and \
                       (last_range < self.blind_zone_range) and \
                       (abs(last_bearing) < goal_handle.request.max_bearing):
                        # CODE WAS ADDED HERE
                        # Compute how long we need to advance the remaining
                        # distance at constant speed, and store the deadline.
                        # The actual movement is done in the loop above.
                        blind_dist = \
                            last_range - goal_handle.request.max_distance
                        blind_end_time = self.get_clock().now() + \
                            Duration(seconds=blind_dist / self.blind_lin_vel)
                        continue
                    # Lost far away or misaligned: real failure.
                    self.vel_cmd.linear.x = 0.0
                    self.vel_cmd.angular.z = 0.0
                    self.vel_pub.publish(self.vel_cmd)
                    if self.sub_markers is not None:
                        self.destroy_subscription(self.sub_markers)
                        self.sub_markers = None
                    goal_handle.abort()
                    self.get_logger().warn(
                        f'{ACTION_NAME}: lost markers, aborting.')
                    return MoveVisualServoing.Result(success=False)

                # Find index of the closest marker
                closest_idx = 0
                closest_range = markers.range[0]
                for i in range(1, markers.num_markers):
                    if markers.range[i] < closest_range:
                        closest_range = markers.range[i]
                        closest_idx = i

                target_range = markers.range[closest_idx]
                target_bearing = markers.bearing[closest_idx]
                
                # Remember last valid detection (used by the blind final approach)
                last_range = target_range
                last_bearing = target_bearing

                # CODE WAS ADDED HERE
                # =====================================================
                # STEP 2: Check success condition
                # Both range and |bearing| must be within the goal limits.
                # =====================================================
                if (target_range < goal_handle.request.max_distance) and \
                   (abs(target_bearing) < goal_handle.request.max_bearing):
                    # Stop the robot
                    self.vel_cmd.angular.z = 0.0
                    self.vel_cmd.linear.x = 0.0
                    self.vel_pub.publish(self.vel_cmd)
                    # Clean up
                    if self.sub_markers is not None:
                        self.destroy_subscription(self.sub_markers)
                        self.sub_markers = None
                    goal_handle.succeed()
                    self.get_logger().info(
                        f'{ACTION_NAME} has succeeded! '
                        f'(range={target_range:.2f}, '
                        f'bearing={target_bearing:.2f})')
                    return MoveVisualServoing.Result(success=True)

                # CODE WAS ADDED HERE
                # =====================================================
                # STEP 3: Compute velocities using P controllers
                # - Angular velocity always tries to align with the marker
                # - Linear velocity only kicks in when reasonably aligned
                #   (same strategy as ActionMove2Pos for consistency)
                # =====================================================
                ang_vel = self.Kp_ang_vel * target_bearing

                if abs(target_bearing) < self.max_angle_to_target:
                    lin_vel = self.Kp_lin_vel * target_range
                    # CODE WAS ADDED HERE
                    # Near the part, limit the speed to the blind approach
                    # speed. This makes the speed at the moment the markers
                    # are lost always the same, so the distance travelled
                    # between the last detection and the start of the blind
                    # phase is constant (instead of varying with each run,
                    # which made the pickup inconsistent).
                    if target_range < self.blind_zone_range:
                        lin_vel = min(lin_vel, self.blind_lin_vel)
                else:
                    lin_vel = 0.0  # rotate in place until aligned enough

                # Clip to safe limits
                lin_vel = clipValue(lin_vel,
                                    -self.MAX_LIN_VEL, self.MAX_LIN_VEL)
                ang_vel = clipValue(ang_vel,
                                    -self.MAX_ANG_VEL, self.MAX_ANG_VEL)

                # CODE WAS ADDED HERE
                # =====================================================
                # STEP 4: Publish velocity commands
                # =====================================================
                self.vel_cmd.linear.x = lin_vel
                self.vel_cmd.angular.z = ang_vel
                self.vel_pub.publish(self.vel_cmd)

                # CODE WAS ADDED HERE
                # =====================================================
                # STEP 5: Publish feedback with current values
                # =====================================================
                feedback.distance = target_range
                feedback.bearing = target_bearing
                goal_handle.publish_feedback(feedback)

    def markersCallback(self, msg: Markers, goal_handle, trigger_event):
        '''
        Receive current robot pose and change its velocity accordingly
        '''
        with self.goal_lock:
            # If the goal is not active, there is nothing to do here
            if not goal_handle.is_active:
                self.get_logger().warn(
                    f'{ACTION_NAME} callback called without active goal!')
                return

            # Store current pose
            self.curr_detected_markers = msg

            # Trigger execute_cb to continue
            trigger_event.set()


def main(args=None):
    ''' Main function - start the action server.
    '''
    rclpy.init(args=args)
    move_vis_serv_action_server = MoveVisualServoingActionServer()

    # Use 2 threads to make sure callbacks can run in parallel and the action
    # does not block.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(move_vis_serv_action_server)
    executor.spin()


if __name__ == '__main__':
    main()
