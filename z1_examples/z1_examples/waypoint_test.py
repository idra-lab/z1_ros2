#!/usr/bin/env python3

# Copyright 2025 IDRA, University of Trento
# Author: Matteo Dalle Vedove (matteodv99tn@gmail.com)
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

import rclpy
import numpy as np
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory



class TrajectoryCaller(Node):

    def __init__(self):
        super().__init__("joint_trajectory_caller")

        self.get_logger().info("Creating client")
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.action_client.wait_for_server()
        self.get_logger().info("Connection established")

    def send_traj_request(self, positions: list[np.ndarray]):
        self.get_logger().info("Creating trajectory request")
        traj_msg = FollowJointTrajectory.Goal()
        traj: JointTrajectory = traj_msg.trajectory
        traj.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]
        for i, pos in enumerate(positions):
            self.get_logger().info(f"{pos}")
            pt = JointTrajectoryPoint()
            pt.positions = pos.tolist()
            pt.time_from_start.sec = (i+1) * 2
            traj.points.append(pt)

        self.get_logger().info("Sending trajectory request")
        return self.action_client.send_goal_async(traj_msg)



def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryCaller()

    future = node.send_traj_request([
        np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        np.array([0.0, 1.0, -1.0, 0.0, 0.0, 0.0]),
        np.array([np.pi / 4, 1.0, -1.0, 0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ])
    rclpy.spin_until_future_complete(node, future)



if __name__ == '__main__':
    main()
