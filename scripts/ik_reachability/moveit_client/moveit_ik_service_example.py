#!/usr/bin/env python3
"""
使用 MoveIt Service 计算 IK (逆运动学)
将 XYZ 位置转换为关节角度
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped


class IKServiceClient(Node):
    """IK Service 客户端"""
    
    def __init__(self):
        super().__init__('ik_service_client')
        
        # 创建 Service Client
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )
        
        # 等待服务可用
        self.get_logger().info("Waiting for IK service...")
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK service not available, waiting...')
        
        self.get_logger().info("IK service is ready!")
    
    def compute_ik(self, x, y, z, group_name="left_arm"):
        """
        计算逆运动学
        
        Args:
            x, y, z: 目标位置 (米)
            group_name: 规划组名称
        
        Returns:
            list: 关节角度列表，如果失败返回 None
        """
        # 创建请求
        request = GetPositionIK.Request()
        
        # 设置规划组
        request.ik_request.group_name = group_name
        
        # 设置目标位置
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world"  # 或 "base_link"
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        
        # 设置姿态（四元数，保持水平）
        pose_stamped.pose.orientation.w = 1.0
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        
        request.ik_request.pose_stamped = pose_stamped
        
        # 设置超时
        request.ik_request.timeout.sec = 5
        
        # 避碰检测
        request.ik_request.avoid_collisions = True
        
        self.get_logger().info(f"Computing IK for position: ({x}, {y}, {z})")
        
        # 调用服务
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            
            # 检查错误代码
            if response.error_code.val == 1:  # SUCCESS
                joint_state = response.solution.joint_state
                
                self.get_logger().info("✅ IK Solution found!")
                self.get_logger().info(f"Joint names: {joint_state.name}")
                self.get_logger().info(f"Joint angles: {joint_state.position}")
                
                return list(joint_state.position)
            else:
                self.get_logger().error(f"❌ IK failed with error code: {response.error_code.val}")
                return None
        else:
            self.get_logger().error("❌ Service call failed!")
            return None


# ============================================================================
# 使用示例
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    
    ik_client = IKServiceClient()
    
    try:
        # 示例 1: 计算左臂的 IK
        print("\n" + "="*60)
        print("示例 1: 左臂 IK")
        print("="*60)
        joint_angles = ik_client.compute_ik(
            x=0.3,
            y=0.2,
            z=0.5,
            group_name="right_arm"
        )
        
        if joint_angles:
            print(f"\n关节角度 (弧度):")
            for i, angle in enumerate(joint_angles[:7]):  # 只显示前7个关节
                print(f"  Joint {i+1}: {angle:.4f} rad ({angle*57.2958:.2f}°)")
        
        # 示例 2: 计算右臂的 IK
        print("\n" + "="*60)
        print("示例 2: 右臂 IK")
        print("="*60)
        joint_angles = ik_client.compute_ik(
            x=2.3,
            y=2.2,
            z=2.5,
            group_name="right_arm"
        )
        
        # 示例 3: 测试不可达位置
        print("\n" + "="*60)
        print("示例 3: 测试不可达位置")
        print("="*60)
        joint_angles = ik_client.compute_ik(
            x=2.0,  # 太远了
            y=0.0,
            z=0.0,
            group_name="right_arm"
        )
        
    except KeyboardInterrupt:
        ik_client.get_logger().info("Interrupted")
    finally:
        ik_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
