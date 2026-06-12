#!/usr/bin/env python3
"""
改进的 MoveIt 控制示例 - 增加调试信息
检测 RViz 干扰并显示详细状态
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
    node.get_logger().info("🚀 启动 MoveIt 示例（带调试信息）...")
    
    # 检查是否有其他 MoveIt 客户端
    node_list = node.get_node_names()
    node.get_logger().info(f"\n📋 当前运行的节点: {len(node_list)} 个")
    for n in node_list:
        if 'rviz' in n.lower() or 'move' in n.lower():
            node.get_logger().warn(f"  ⚠️  检测到: {n}")
    
    # 创建回调组
    callback_group = ReentrantCallbackGroup()
    
    # 初始化 MoveIt2 接口（左臂）
    node.get_logger().info("\n🔧 初始化 MoveIt2 接口...")
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
        end_effector_name="openarm_left_hand",
        group_name="left_arm",
        callback_group=callback_group
    )
    
    # 创建 executor 并在后台运行
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        node.get_logger().info("\n⏳ 等待 MoveIt 初始化...")
        for i in range(5, 0, -1):
            node.get_logger().info(f"   {i} 秒...")
            time.sleep(1.0)
        
        # 示例 1: 移动到 Home 姿态
        node.get_logger().info("\n" + "="*60)
        node.get_logger().info("📍 示例 1: 移动到 Home 姿态 (所有关节归零)")
        node.get_logger().info("="*60)
        
        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        node.get_logger().info("🎯 目标关节角度: [0, 0, 0, 0, 0, 0, 0]")
        node.get_logger().info("🚀 开始规划和执行...")
        
        moveit2.move_to_configuration(home_joints)
        node.get_logger().info("⏳ 等待执行完成...")
        moveit2.wait_until_executed()
        
        node.get_logger().info("✅ Home 姿态到达！")
        
        time.sleep(3.0)
        
        # 示例 2: 移动到特定位置（XYZ坐标）
        node.get_logger().info("\n" + "="*60)
        node.get_logger().info("📍 示例 2: 移动到 XYZ 位置")
        node.get_logger().info("="*60)
        
        position = Point()
        position.x = 0.3
        position.y = 0.2
        position.z = 0.5
        
        node.get_logger().info(f"🎯 目标位置: x={position.x}, y={position.y}, z={position.z}")
        
        # 四元数 [x, y, z, w] - 保持水平姿态
        quat_xyzw = [0.0, 0.0, 0.0, 1.0]
        
        node.get_logger().info("🚀 开始规划和执行（笛卡尔模式）...")
        
        moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw,
            cartesian=False  # False = 关节空间规划
        )
        
        node.get_logger().info("⏳ 等待执行完成...")
        moveit2.wait_until_executed()
        
        node.get_logger().info("✅ 目标位置到达！")
        
        time.sleep(3.0)
        
        # 示例 3: 回到 Home
        node.get_logger().info("\n" + "="*60)
        node.get_logger().info("📍 示例 3: 返回 Home 姿态")
        node.get_logger().info("="*60)
        
        node.get_logger().info("🚀 返回 Home...")
        moveit2.move_to_configuration(home_joints)
        moveit2.wait_until_executed()
        node.get_logger().info("✅ 返回完成！")
        
        node.get_logger().info("\n" + "="*60)
        node.get_logger().info("🎉 所有示例完成！")
        node.get_logger().info("="*60)
        
    except KeyboardInterrupt:
        node.get_logger().info("\n⚠️  被用户中断")
    except Exception as e:
        node.get_logger().error(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        node.get_logger().info("\n🛑 关闭节点...")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
