#!/usr/bin/env python3
# Copyright 2024 TIER IV, Inc. All rights reserved.
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

# cspell: ignore atteuler

from autoware_sensing_msgs.msg import GnssInsOrientationStamped
from geometry_msgs.msg import Quaternion
import numpy as np
import rclpy
from rclpy.node import Node
from septentrio_gnss_driver.msg import AttEuler


class OrientationConverter(Node):
    def __init__(self):
        super().__init__("septentrio_orientation_converter")
        self.publisher = self.create_publisher(
            GnssInsOrientationStamped, "/sensing/gnss/septentrio/orientation", 10
        )
        self.subscription = self.create_subscription(
            AttEuler, "/sensing/gnss/septentrio/atteuler", self.attitude_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def heading_to_quaternion(self, heading: float) -> Quaternion:
        # The input heading is in a North-East coordinate system and measured in degrees.
        # Heading values range from [0, 360).
        # Examples:
        # - Heading is   0[deg] when the vehicle faces North.
        # - Heading is  90[deg] when the vehicle faces East.
        # - Heading is 180[deg] when the vehicle faces South.
        # - Heading is 270[deg] when the vehicle faces West.

        # The output quaternion represents orientation in an East-North-Up (ENU) coordinate system.
        # The quaternion is of the form [x, y, z, w], where:
        # - Facing East:  [x, y, z, w] = [0, 0,  0,   1]   = [0, 0, sin(  0[deg]), cos(  0[deg])]
        # - Facing North: [x, y, z, w] = [0, 0,  0.7, 0.7] = [0, 0, sin( 45[deg]), cos( 45[deg])]
        # - Facing West:  [x, y, z, w] = [0, 0,  1,   0]   = [0, 0, sin( 90[deg]), cos( 90[deg])]
        # - Facing South: [x, y, z, w] = [0, 0, -0.7, 0.7] = [0, 0, sin(135[deg]), cos(135[deg])]
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = np.sin(np.deg2rad(90 - heading) / 2.0)
        q.w = np.cos(np.deg2rad(90 - heading) / 2.0)
        return q

    def attitude_callback(self, attitude_msg: AttEuler) -> None:
        # When septentrio driver cannot measure the heading, it will publish -20000000000.0
        if attitude_msg.heading < 0:
            return

        # When heading is NaN, it means the heading is not available.
        if np.isnan(attitude_msg.heading):
            return

        orientation_msg = GnssInsOrientationStamped()
        orientation_msg.header = attitude_msg.header

        # Even if using dual antenna, the roll is not estimated by the septentrio driver.
        # Therefore, this assume the roll & pitch are 0 and convert the heading to quaternion.
        orientation_msg.orientation.orientation = self.heading_to_quaternion(attitude_msg.heading)

        # Septentrio driver does not provide the covariance of the heading.
        # Therefore, this assumes the covariance of the heading is 1.0.
        orientation_msg.orientation.rmse_rotation_x = 1.0
        orientation_msg.orientation.rmse_rotation_y = 1.0
        orientation_msg.orientation.rmse_rotation_z = 1.0

        self.publisher.publish(orientation_msg)


def main(args=None) -> None:
    rclpy.init(args=args)

    converter = OrientationConverter()

    rclpy.spin(converter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
