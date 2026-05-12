#!/usr/bin/env python3
"""
MoveIt ROS2 Action 接口示例
展示如何使用底层的 ROS2 Action 直接调用 MoveIt
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from geometry_msgs.msg import PoseStamped
import time


class MoveItActionClient(Node):
    """使用 ROS2 Action 调用 MoveIt"""
    
    def __init__(self):
        super().__init__('moveit_action_client')
        
        # 创建 Action Client
        self._action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'  # MoveIt 的默认 action 名称
        )
        
        self.get_logger().info("Waiting for MoveIt action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Connected to MoveIt action server!")
    
    def send_goal(self, x=0.3, y=0.2, z=0.5):
        """
        使用 ROS2 Action 发送目标位置
        这是底层接口，类似于直接调用 ros2 action
        """
        # 创建 MoveGroup Goal
        goal_msg = MoveGroup.Goal()
        
        # 设置规划请求
        goal_msg.request.group_name = "left_arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        # 设置目标位置约束
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "world"
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z
        pose_goal.pose.orientation.w = 1.0
        
        # 添加到约束中
        # 注意：完整的实现需要设置 PositionConstraint 和 OrientationConstraint
        
        self.get_logger().info(f"Sending goal: x={x}, y={y}, z={z}")
        
        # 发送目标
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Goal 响应回调"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted! Waiting for result...')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """执行结果回调"""
        result = future.result().result
        
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Motion planning and execution SUCCEEDED!')
        else:
            self.get_logger().error(f'Motion planning FAILED with error code: {result.error_code.val}')
    
    def feedback_callback(self, feedback_msg):
        """执行过程反馈"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.state}')


# ============================================================================
# 使用方式 (与 ros2 action 类似)
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    
    action_client = MoveItActionClient()
    
    try:
        # 发送目标
        action_client.send_goal(x=0.3, y=0.2, z=0.5)
        
        # 保持节点运行
        rclpy.spin(action_client)
        
    except KeyboardInterrupt:
        action_client.get_logger().info("Interrupted")
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
