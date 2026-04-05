#!/usr/bin/env python3

# Copyright 2025 Tier IV, Inc. All rights reserved.
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

import threading

from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
from rclpy.node import Node


class DiagnosticPublisher(Node):
    def __init__(self):
        super().__init__("multi_diagnostic_publisher")

        self.diagnostic_names = [
            "injection_stop_available",
            "injection_local_available",
            "injection_remote_available",
            "injection_autonomous_available",
            "injection_comfortable_stop_available",
            "injection_main_ecu_in_lane_moderate_stop_available",
            "injection_main_ecu_in_lane_emergency_stop_available",
        ]
        self.status_states = {name: (DiagnosticStatus.OK, "OK") for name in self.diagnostic_names}

        # Time publisher map: diagnostic name → Publisher
        self.time_publishers = {}
        for name in self.diagnostic_names:
            topic_suffix = name.replace("injection_", "").replace("_available", "")
            topic_name = f"/system/debug/injection/{topic_suffix}"
            self.time_publishers[name] = self.create_publisher(Time, topic_name, 10)
        self.publish_time_flags = {name: None for name in self.diagnostic_names}

        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        threading.Thread(target=self.input_listener, daemon=True).start()

    def timer_callback(self):
        msg = DiagnosticArray()
        for name, (level, message) in self.status_states.items():
            now = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = name
            status.level = level
            status.message = message
            status.hardware_id = "dummy_hardware"
            msg.header.stamp = now
            msg.status.append(status)

            if self.publish_time_flags[name]:
                time_pub = self.time_publishers[name]
                time_pub.publish(now)
                self.publish_time_flags[name] = False  # reset flag
        self.diagnostics_pub.publish(msg)

    def input_listener(self):
        while True:
            try:
                print("\nInput number for switching diag state (OK <-> Error):")
                for i, name in enumerate(self.diagnostic_names):
                    current_level, _ = self.status_states[name]
                    print(f" {i}: {name} (level {current_level})")
                user_input = input("Input: ").strip()
                index = int(user_input)
                if 0 <= index < len(self.diagnostic_names):
                    name = self.diagnostic_names[index]
                    current_level, _ = self.status_states[name]
                    new_level = (
                        DiagnosticStatus.ERROR
                        if current_level == DiagnosticStatus.OK
                        else DiagnosticStatus.OK
                    )
                    new_message = "Error" if new_level == DiagnosticStatus.ERROR else "OK"

                    if current_level == DiagnosticStatus.OK and new_level == DiagnosticStatus.ERROR:
                        self.publish_time_flags[name] = True

                    self.status_states[name] = (new_level, new_message)
                    print(f"[Switched!] {name} → {new_message}")
                else:
                    print("Invalid Number. Please try again.")
            except Exception as e:
                print(f"Input Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
