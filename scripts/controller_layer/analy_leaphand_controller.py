import argparse
import sys
import math
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import pandas as pd
import datetime
import os
import signal

class HandTimeDomainAnalyzer(Node):
    def __init__(self, mode='action', cmd_topic=None, joint_names_list=[]):
        super().__init__('hand_analyzer')
        self.mode = mode
        self.cmd_topic = cmd_topic

        # raw data collected as list of dicts
        self.raw_records = []

        # Initialize joint_names based on mode
        if joint_names_list:
            # User provided joint names (for topic mode)
            self.joint_names = list(joint_names_list)
        elif mode == 'topic':
            # Default LeapHand joint names for topic mode
            self.joint_names = ['right_index_mcp_side', 'right_index_mcp_forward', 'right_index_pip', 'right_index_dip',
                        'right_middle_mcp_side', 'right_middle_mcp_forward', 'right_middle_pip', 'right_middle_dip',
                        'right_ring_mcp_side', 'right_ring_mcp_forward', 'right_ring_pip', 'right_ring_dip',
                        'right_thumb_mcp_side', 'right_thumb_mcp_forward', 'right_thumb_pip_joint', 'right_thumb_dip_joint']
        else:
            # Action mode: joint names will be extracted from controller_state message
            self.joint_names = []
        
        self.start_time = None

        # store latest joint_states for mapping actual positions when commands arrive via topic
        self.latest_joint_state = {}

        # directory setup
        self.today_str = datetime.datetime.now().strftime('%Y%m%d')
        self.base_dir = "analy_leaphand_controller"
        self.csv_dir = os.path.join(self.base_dir, 'csv', self.today_str)
        self.img_dir = os.path.join(self.base_dir, 'img', self.today_str)

        os.makedirs(self.csv_dir, exist_ok=True)
        os.makedirs(self.img_dir, exist_ok=True)

        # Subscriptions differ by mode
        if self.mode == 'action':
            # action controller state contains reference, feedback and error
            self.get_logger().info('Mode=action: subscribing to controller_state topic')
            self.sub = self.create_subscription(
                JointTrajectoryControllerState,
                '/right_hand_controller/controller_state',
                self.callback,
                10)
        else:
            # topic mode: subscribe to a command topic providing reference positions
            topic = self.cmd_topic or '/right_hand_forward_position_controller/commands'
            self.get_logger().info(f"Mode=topic: subscribing to command topic '{topic}'")
            self.sub_cmd = self.create_subscription(
                Float64MultiArray,
                topic,
                self.cmd_callback,
                10)

        # always subscribe to /joint_states to get actual positions for topic mode
        self.sub_js = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10)

        self.get_logger().info('Analyzer started — waiting for data...')

    def callback(self, msg):
        try:
            if not self.joint_names:
                self.joint_names = msg.joint_names

            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self.start_time is None:
                self.start_time = t
            
            rel_time = t - self.start_time
            
            record = {'time': rel_time}
            for i, name in enumerate(self.joint_names):
                record[f'{name}_ref'] = msg.reference.positions[i]
                record[f'{name}_act'] = msg.feedback.positions[i]
                record[f'{name}_err'] = msg.error.positions[i]
            
            self.raw_records.append(record)
        except Exception:
            pass

    def joint_state_cb(self, msg: JointState):
        # cache latest joint positions by name
        try:
            for i, name in enumerate(msg.name):
                # some JointState messages may have fewer entries; guard access
                if i < len(msg.position):
                    self.latest_joint_state[name] = msg.position[i]
        except Exception:
            pass

    def cmd_callback(self, msg: Float64MultiArray):
        # handle incoming reference positions from a topic (topic mode)
        try:
            if not self.joint_names:
                self.get_logger().warn('Joint names not set for topic mode — provide --joint-names')
                return

            now = datetime.datetime.now().timestamp()
            if self.start_time is None:
                self.start_time = now
            rel_time = now - self.start_time

            record = {'time': rel_time}
            data = list(msg.data)

            # map incoming array positions to configured joint names
            for i, name in enumerate(self.joint_names):
                ref = data[i] if i < len(data) else float('nan')
                act = self.latest_joint_state.get(name, float('nan'))
                record[f'{name}_ref'] = ref
                record[f'{name}_act'] = act
                # compute error when actual exists
                try:
                    record[f'{name}_err'] = (ref - act) if (not math.isnan(ref) and not math.isnan(act)) else float('nan')
                except Exception:
                    record[f'{name}_err'] = float('nan')

            self.raw_records.append(record)
        except Exception:
            pass

    def save_and_plot(self):
        if not self.raw_records:
            self.get_logger().warn('No data collected.')
            return

        self.get_logger().info('Calculating errors and plotting...')
        df = pd.DataFrame(self.raw_records)
        timestamp = datetime.datetime.now().strftime("%H%M%S")

        # 1. Save CSV
        csv_filename = os.path.join(self.csv_dir, f"hand_data_{timestamp}.csv")
        df.to_csv(csv_filename, index=False)
        self.get_logger().info(f'CSV saved: {csv_filename}')

        # 2. Plot (4x4 grid)
        target_joints = list(range(min(16, len(self.joint_names))))
        fig, axes = plt.subplots(4, 4, figsize=(24, 18))
        axes = axes.flatten()

        for idx, j_idx in enumerate(target_joints):
            name = self.joint_names[j_idx]
            ref = df[f'{name}_ref']
            act = df[f'{name}_act']
            err = df[f'{name}_err']
            
            # Calculate max error (absolute value)
            max_err = err.abs().max()

            # Plot curves
            axes[idx].plot(df['time'], ref, 'r--', linewidth=1.5, label='Cmd (Ref)')
            axes[idx].plot(df['time'], act, 'b-', linewidth=1.2, label='Actual (Act)')
            axes[idx].plot(df['time'], err, 'g-', alpha=0.6, label='Error')
            
            axes[idx].set_title(f'{name}\nMax Err: {max_err:.4f} rad', fontsize=11, fontweight='bold')
            axes[idx].set_xlabel('Time (s)')
            axes[idx].set_ylabel('Pos (rad)')
            axes[idx].grid(True, linestyle=':', alpha=0.6)
            
            if idx == 0:
                axes[idx].legend(loc='upper right', fontsize='small')

        plt.suptitle(f"LeapHand Performance Analysis - {self.today_str}_{timestamp}\n(Red: Cmd, Blue: Actual, Green: Error)", fontsize=20)
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        
        # Save image
        img_filename = os.path.join(self.img_dir, f"plot_error_{timestamp}.png")
        plt.savefig(img_filename, dpi=120)
        self.get_logger().info(f'Analysis plot saved: {img_filename}')
        plt.show()

def main(args=None):
    parser = argparse.ArgumentParser(description='LeapHand controller analyzer')
    parser.add_argument('--mode', choices=['action', 'topic'], default='action',
                        help='Data source mode: action (controller_state) or topic (external commands)')
    parser.add_argument('--cmd-topic', default=None,
                        help='Command topic for topic mode (Float64MultiArray)')
    parser.add_argument('--joint-names', default=[],
                        help="Comma-separated joint names for topic mode (e.g. 'joint1,joint2,...')")
    parsed = parser.parse_args(args=args)

    
    if parsed.joint_names:
        joint_names_list = [s.strip() for s in parsed.joint_names.split(',') if s.strip()]


    rclpy.init(args=args)
    node = HandTimeDomainAnalyzer(mode=parsed.mode, cmd_topic=parsed.cmd_topic)

    try:
        # use spin to receive callbacks until interruption
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info('Stopping listener and producing report...')
    finally:
        node.save_and_plot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()