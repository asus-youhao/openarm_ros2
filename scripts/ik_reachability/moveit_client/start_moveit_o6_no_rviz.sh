#!/bin/bash
# MoveIt 启动脚本（无 RViz）- OpenArm V10 + O6 Hands
# 适合配合 Python 脚本使用

echo "🚀 启动 MoveIt (OpenArm V10 + O6 Hands, 无 RViz)"
echo "================================================"

# 加载工作空间环境
cd ~/ros2_ws_yh
source install/setup.bash
echo "✅ 环境已加载"

# 清理之前的进程
if pgrep -f "ros2_control_node|move_group" > /dev/null; then
    echo "⚠️  检测到旧进程，正在清理..."
    pkill -f "ros2_control_node|move_group" 2>/dev/null
    sleep 2
fi

# 先启动硬件和控制器
echo "步骤 1: 启动硬件接口和控制器..."
ros2 launch openarm_bringup openarm_o6_bimanual.launch.py \
  right_can_interface:=can2 \
  left_can_interface:=can3 \
  right_o6_can_interface:=can0 \
  left_o6_can_interface:=can1 \
  use_fake_hardware:=false \
  launch_rviz:=false &

BRINGUP_PID=$!
sleep 6

# 然后启动 MoveIt move_group
echo ""
echo "步骤 2: 启动 MoveIt move_group 节点（Planning Only）..."
ros2 launch openarm_bimanual_moveit_config move_group_only.launch.py &

MOVEGROUP_PID=$!
sleep 3

echo ""
echo "✅ MoveIt 启动完成（OpenArm V10 + O6 Hands）！"
echo "================================================"
echo "⚙️  系统架构："
echo "  • Hardware 层: 管理 Arm (7 DOF) + O6 Hand (6 DOF)"
echo "  • MoveIt 层: 只规划 Arm 运动（不控制 O6 手指）"
echo ""
echo "现在可以运行 Python 脚本（只控制手臂）："
echo "  python3 moveit_simple_example.py"
echo "  python3 moveit_debug_example.py"
echo ""
echo "📡 CAN 接口："
echo "  右臂: can2  |  左臂: can3"
echo "  右手: can0  |  左手: can1"
echo ""
echo "💡 提示: O6 Hand 需要通过独立的 action client 控制"
echo "  ros2 action send_goal /right_hand_controller/follow_joint_trajectory ..."
echo ""
echo "按 Ctrl+C 停止所有节点"
echo "================================================"

# 等待用户中断
trap "echo '停止中...'; kill $BRINGUP_PID $MOVEGROUP_PID 2>/dev/null; exit" INT
wait
