#!/usr/bin/env python3
"""
简单的 MoveIt 控制示例 - 使用 PyMoveIt2
最小化示例，展示基本的位置控制
"""

import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Point
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import time


def main(args=None):
    rclpy.init(args=args)
    
    # 创建节点
    node = Node('simple_moveit_example')
    node.get_logger().info("Starting simple MoveIt example...")
    
    # 创建回调组
    callback_group = ReentrantCallbackGroup()
    
    # 初始化 MoveIt2 接口（左臂）
    moveit2 = MoveIt2(
        node=node,
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
        end_effector_name="openarm_left_link7",  # actual EE link in URDF (SRDF: parent_link of left_ee)
        group_name="left_arm",
        callback_group=callback_group
    )
    
    # 创建 executor 并在后台运行
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        node.get_logger().info("Waiting for MoveIt to initialize...")
        time.sleep(3.0)
        
        # 示例 1: 移动到 Home 姿态
        node.get_logger().info("\n示例 1: 移动到 Home 姿态 (所有关节归零)")
        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        moveit2.move_to_configuration(home_joints)
        moveit2.wait_until_executed()
        node.get_logger().info("Home 姿态到达")
        
        time.sleep(2.0)
        
        # 示例 2: 移动到特定位置（XYZ坐标）
        node.get_logger().info("\n示例 2: 移动到 XYZ 位置 (0.3, 0.2, 0.5)")
        
        position = Point()
        position.x = 0.3
        position.y = 0.2
        position.z = 0.5
        
        # 四元数 [x, y, z, w] - 保持水平姿态
        quat_xyzw = [0.0, 0.0, 0.0, 1.0]
        
        moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw,
            cartesian=False  # False = 关节空间规划, True = 笛卡尔规划
        )
        
        moveit2.wait_until_executed()
        node.get_logger().info("目标位置到达")
        
        time.sleep(2.0)
        
        # 示例 3: 回到 Home
        node.get_logger().info("\n示例 3: 返回 Home")
        moveit2.move_to_configuration(home_joints)
        moveit2.wait_until_executed()
        node.get_logger().info("返回完成")
        
        node.get_logger().info("\n所有示例完成！")
        
    except KeyboardInterrupt:
        node.get_logger().info("被用户中断")
    except Exception as e:
        node.get_logger().error(f"错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
