#!/usr/bin/env python3

import threading

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
from rclpy.node import Node


class DiagnosticPublisher(Node):
    def __init__(self):
        super().__init__("multi_diagnostic_publisher")

        # 診断名リスト
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
        self.publisher_ = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        threading.Thread(target=self.input_listener, daemon=True).start()

    def timer_callback(self):
        msg = DiagnosticArray()
        for name, (level, message) in self.status_states.items():
            status = DiagnosticStatus()
            status.name = name
            status.level = level
            status.message = message
            status.hardware_id = "dummy_hardware"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.status.append(status)
        self.publisher_.publish(msg)

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
