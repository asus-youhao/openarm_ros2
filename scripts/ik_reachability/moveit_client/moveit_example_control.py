#!/usr/bin/env python3
"""
MoveIt 控制示例：展示如何使用 PyMoveIt2 控制机械臂
包含：位置控制、关节空间控制、笛卡尔路径规划
使用 PyMoveIt2 - ROS2 Humble 的标准 Python 接口
"""

import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Point
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import time


class MoveItController(Node):
    """MoveIt 控制器示例 (使用 PyMoveIt2)"""
    
    def __init__(self):
        super().__init__('moveit_controller_example')
        
        # 创建回调组
        callback_group = ReentrantCallbackGroup()
        
        # 初始化 MoveIt2 接口
        self.get_logger().info("Initializing PyMoveIt2...")
        
        # 左臂 MoveIt2 接口
        self.left_arm = MoveIt2(
            node=self,
            joint_names=[
                'openarm_left_joint1',
                'openarm_left_joint2', 
                'openarm_left_joint3',
                'openarm_left_joint4',
                'openarm_left_joint5',
                'openarm_left_joint6',
                'openarm_left_joint7'
            ],
            base_link_name="world",
            end_effector_name="openarm_left_hand",
            group_name="left_arm",
            callback_group=callback_group
        )
        
        # 右臂 MoveIt2 接口
        self.right_arm = MoveIt2(
            node=self,
            joint_names=[
                'openarm_right_joint1',
                'openarm_right_joint2',
                'openarm_right_joint3',
                'openarm_right_joint4',
                'openarm_right_joint5',
                'openarm_right_joint6',
                'openarm_right_joint7'
            ],
            base_link_name="world",
            end_effector_name="openarm_right_hand",
            group_name="right_arm",
            callback_group=callback_group
        )
        
        self.get_logger().info("PyMoveIt2 initialized successfully!")
    
    def move_to_pose(self, group_name="left_arm", x=0.3, y=0.2, z=0.5):
        """移动到指定的 xyz 位置"""
        self.get_logger().info(f"Moving {group_name} to position: x={x}, y={y}, z={z}")
        
        arm = self.left_arm if group_name == "left_arm" else self.right_arm
        
        position = Point()
        position.x = x
        position.y = y
        position.z = z
        
        quat_xyzw = [0.0, 0.0, 0.0, 1.0]
        
        arm.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=False)
        arm.wait_until_executed()
        
        self.get_logger().info("Motion completed!")
        return True
    
    def move_to_joint_angles(self, group_name="left_arm", joint_angles=None):
        """移动到指定的关节角度"""
        if joint_angles is None:
            joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.get_logger().info(f"Moving {group_name} to joint angles: {joint_angles}")
        
        arm = self.left_arm if group_name == "left_arm" else self.right_arm
        arm.move_to_configuration(joint_angles)
        arm.wait_until_executed()
        
        self.get_logger().info("Joint motion completed!")
        return True


def main(args=None):
    rclpy.init(args=args)
    
    controller = MoveItController()
    
    # 创建 executor 来运行节点
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)
    
    # 在后台线程中 spin
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        # 等待 MoveIt 初始化
        controller.get_logger().info("Waiting for MoveIt to initialize...")
        time.sleep(3.0)
        
        # 示例 1: 移动到特定位置
        controller.get_logger().info("\n=== Example 1: Move to XYZ position ===")
        controller.move_to_pose("left_arm", x=0.3, y=0.2, z=0.5)
        
        time.sleep(2.0)
        
        # 示例 2: 移动到 home 姿态
        controller.get_logger().info("\n=== Example 2: Move to home position ===")
        controller.move_to_joint_angles("left_arm", [0.0]*7)
        
        controller.get_logger().info("\n✅ All examples completed!")
        
    except KeyboardInterrupt:
        controller.get_logger().info("Interrupted by user")
    except Exception as e:
        controller.get_logger().error(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        executor.shutdown()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
