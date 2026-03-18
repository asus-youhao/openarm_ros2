#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

class SimpleGripperClient(Node):
    def __init__(self):
        super().__init__('simple_gripper_client')
        self._client = ActionClient(self, GripperCommand, '/right_gripper_controller/gripper_cmd')

    def send_gripper_goal(self, position, max_effort=50.0):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort
        self._client.wait_for_server()
        self._client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGripperClient()
    print("Press 1 to open gripper (0.044m), q to quit.")
    try:
        while rclpy.ok():
            cmd = input("Input: ").strip()
            if cmd == '1':
                node.send_gripper_goal(0.044)  # open to 44mm (typical max opening for a gripper)
                print("Sent gripper open command.")
            elif cmd == '2':
                node.send_gripper_goal(0.0)  # close gripper
                print("Sent gripper close command.")
            elif cmd == 'q':
                break
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()