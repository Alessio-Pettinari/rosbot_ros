# Copyright 2021 Open Source Robotics Foundation, Inc.
# Copyright 2023 Husarion
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

from threading import Event
from threading import Thread

from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations


class SimulationTestNode(Node):
    __test__ = False
    XY_TOLERANCE = 0.05
    YAW_TOLERANCE = 0.1

    def __init__(self, name="test_node"):
        super().__init__(name)
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_yaw = 0.0

        self.goal_x_event = Event()
        self.goal_y_event = Event()
        self.goal_yaw_event = Event()

    def clear_events(self):
        self.goal_x_event.clear()
        self.goal_y_event.clear()
        self.goal_yaw_event.clear()

    def set_destination_speed(self, v_x, v_y, v_yaw):
        self.clear_events()
        self.v_x = v_x
        self.v_y = v_y
        self.v_yaw = v_yaw

    def create_test_subscribers_and_publishers(self):
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odometry_callback, 10
        )
        self.timer = None

    def start_node_thread(self):
        self.ros_spin_thread = Thread(
            target=lambda node: rclpy.spin(node), args=(self,)
        )
        self.ros_spin_thread.start()
        self.timer = self.create_timer(1.0 / 10.0, self.publish_cmd_vel_messages)

    def odometry_callback(self, data: Odometry):
        twist = data.twist.twist

        if abs(twist.linear.x - self.v_x) < self.XY_TOLERANCE:
            self.goal_x_event.set()

        if abs(twist.linear.y - self.v_y) < self.XY_TOLERANCE:
            self.goal_y_event.set()

        if abs(twist.angular.z - self.v_yaw) < self.YAW_TOLERANCE:
            self.goal_yaw_event.set()

    def publish_cmd_vel_messages(self):
        twist_msg = Twist()

        twist_msg.linear.x = self.v_x
        twist_msg.linear.y = self.v_y
        twist_msg.angular.z = self.v_yaw

        self.cmd_vel_publisher.publish(twist_msg)
