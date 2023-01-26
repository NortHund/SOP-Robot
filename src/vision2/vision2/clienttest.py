# Copyright 2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from action_msgs.msg import GoalStatus

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from std_msgs.msg import String
from vision2_msgs.msg import Faces, Face, Point2, FaceImage, FaceImages

#import actionlib

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from builtin_interfaces.msg import Duration


class MinimalActionClient(Node):

    def __init__(self):
        super().__init__('minimal_action_client')
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback))

    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info('Goal failed')

        # Shutdown after receiving a result
        rclpy.shutdown()

    def send_goal(self):
        goal = [0.6, 0.0, 0.0, 0.0]
    
        head_joints = ['head_pan_joint', 
                            'head_tilt_right_joint', 
                            'head_tilt_left_joint', 
                            'head_tilt_vertical_joint']

        duration = Duration(sec=1,nanosec=0)
        msg1 = [JointTrajectoryPoint(positions=goal,time_from_start=duration)]
        msg2 = JointTrajectory(joint_names=head_joints,points=msg1)
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = msg2
    
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        #goal_msg = Fibonacci.Goal()
        #goal_msg.order = 10
        
        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    rclpy.init(args=args)

    action_client = MinimalActionClient()

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
