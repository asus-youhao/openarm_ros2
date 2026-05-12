#!/bin/bash
# MoveIt 命令行使用示例指南
# 展示如何使用 ros2 命令直接与 MoveIt 交互

echo "=============================================="
echo "  MoveIt 命令行使用指南"
echo "=============================================="

# ============================================================================
# 1. 查看可用的 MoveIt Actions
# ============================================================================
echo -e "\n📌 1. 查看 MoveIt 的 Action 接口："
echo "ros2 action list | grep move"
echo ""
echo "常见的 MoveIt Actions:"
echo "  /move_action                  - 主要的运动规划和执行 action"
echo "  /execute_trajectory           - 执行已规划好的轨迹"
echo ""

# ============================================================================
# 2. 查看 MoveIt Services
# ============================================================================
echo -e "\n📌 2. 查看 MoveIt 的 Service 接口："
echo "ros2 service list | grep move"
echo ""
echo "常见的 MoveIt Services:"
echo "  /compute_ik                   - 计算逆运动学"
echo "  /compute_fk                   - 计算正运动学"
echo "  /get_planning_scene           - 获取规划场景"
echo "  /apply_planning_scene         - 应用规划场景"
echo ""

# ============================================================================
# 3. 使用 ROS2 Service 计算 IK (逆运动学)
# ============================================================================
echo -e "\n📌 3. 计算 IK (Inverse Kinematics) - 将 XYZ 转换为关节角度："
echo ""
cat << 'EOF'
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "
ik_request:
  group_name: 'left_arm'
  pose_stamped:
    header:
      frame_id: 'world'
    pose:
      position:
        x: 0.3
        y: 0.2
        z: 0.5
      orientation:
        w: 1.0
        x: 0.0
        y: 0.0
        z: 0.0
"
EOF
echo ""

# ============================================================================
# 4. 使用 ROS2 Service 计算 FK (正运动学)
# ============================================================================
echo -e "\n📌 4. 计算 FK (Forward Kinematics) - 将关节角度转换为 XYZ："
echo ""
cat << 'EOF'
ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "
header:
  frame_id: 'world'
fk_link_names: ['openarm_left_hand']
robot_state:
  joint_state:
    name: [
      'openarm_left_joint1',
      'openarm_left_joint2',
      'openarm_left_joint3',
      'openarm_left_joint4',
      'openarm_left_joint5',
      'openarm_left_joint6',
      'openarm_left_joint7'
    ]
    position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
"
EOF
echo ""

# ============================================================================
# 5. 使用 ROS2 Action 执行运动规划
# ============================================================================
echo -e "\n📌 5. 使用 Action 执行完整的运动规划和控制："
echo ""
cat << 'EOF'
ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "
request:
  group_name: 'left_arm'
  num_planning_attempts: 5
  allowed_planning_time: 5.0
  max_velocity_scaling_factor: 0.5
  max_acceleration_scaling_factor: 0.5
  goal_constraints:
    - position_constraints:
        - header:
            frame_id: 'world'
          link_name: 'openarm_left_hand'
          target_point_offset:
            x: 0.0
            y: 0.0
            z: 0.0
          constraint_region:
            primitive_poses:
              - position:
                  x: 0.3
                  y: 0.2
                  z: 0.5
                orientation:
                  w: 1.0
" --feedback
EOF
echo ""

# ============================================================================
# 6. Python API 方式 (推荐)
# ============================================================================
echo -e "\n📌 6. Python API 方式 (最简单，推荐)："
echo ""
echo "方式 A: 使用 moveit_commander (ROS2 Humble 之前)"
cat << 'EOF'
from moveit_commander import MoveGroupCommander

# 初始化
left_arm = MoveGroupCommander("left_arm")

# 设置目标位置
left_arm.set_position_target([0.3, 0.2, 0.5])

# 规划并执行
plan = left_arm.plan()
left_arm.execute(plan[1])
EOF
echo ""

echo "方式 B: 使用 MoveItPy (ROS2 Humble+, 推荐)"
cat << 'EOF'
from moveit.planning import MoveItPy

# 初始化
moveit = MoveItPy(node_name="moveit_node")
left_arm = moveit.get_planning_component("left_arm")

# 设置目标并执行
left_arm.set_goal_state(pose_stamped_msg=pose_goal)
plan_result = left_arm.plan()
left_arm.execute(plan_result.trajectory)
EOF
echo ""

# ============================================================================
# 总结对比
# ============================================================================
echo -e "\n=============================================="
echo "  方式对比总结"
echo "=============================================="
echo ""
echo "┌────────────────────┬──────────────┬──────────────┬──────────────┐"
echo "│ 方式               │ 难度         │ 灵活性       │ 推荐程度     │"
echo "├────────────────────┼──────────────┼──────────────┼──────────────┤"
echo "│ Python API         │ ⭐           │ ⭐⭐⭐⭐⭐   │ ⭐⭐⭐⭐⭐   │"
echo "│ ROS2 Service (IK)  │ ⭐⭐         │ ⭐⭐⭐       │ ⭐⭐⭐       │"
echo "│ ROS2 Action        │ ⭐⭐⭐       │ ⭐⭐⭐⭐     │ ⭐⭐⭐       │"
echo "│ 命令行 (调试用)    │ ⭐⭐⭐⭐     │ ⭐⭐         │ ⭐⭐         │"
echo "└────────────────────┴──────────────┴──────────────┴──────────────┘"
echo ""

# ============================================================================
# 实际使用建议
# ============================================================================
echo -e "\n✨ 实际使用建议："
echo ""
echo "1. 快速测试/调试：使用 ros2 service call 计算 IK"
echo "   → 快速验证某个位置是否可达"
echo ""
echo "2. 脚本/程序开发：使用 Python MoveItPy API"
echo "   → 最简单、最强大、文档最全"
echo ""
echo "3. 底层控制/集成：使用 ROS2 Action"
echo "   → 需要更精细的控制或与其他系统集成"
echo ""
echo "4. 命令行快速测试："
echo "   → 查看接口、调试网络、验证配置"
echo ""

echo -e "\n=============================================="
echo "  示例脚本已创建"
echo "=============================================="
echo ""
echo "Python API 示例:"
echo "  ./scripts/moveit_example_control.py"
echo ""
echo "Action 接口示例:"
echo "  ./scripts/moveit_action_example.py"
echo ""
