import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
import matplotlib.pyplot as plt
import pandas as pd
import datetime
import os
import signal

class HandTimeDomainAnalyzer(Node):
    def __init__(self):
        super().__init__('hand_analyzer')
        self.sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/right_hand_controller/controller_state',
            self.callback,
            10)
        
        self.raw_records = []
        self.joint_names = []
        self.start_time = None
        
        # 資料夾設定
        self.today_str = datetime.datetime.now().strftime('%Y%m%d')
        self.base_dir = "analy_leaphand_controller"
        self.csv_dir = os.path.join(self.base_dir, 'csv', self.today_str)
        self.img_dir = os.path.join(self.base_dir, 'img', self.today_str)
        
        os.makedirs(self.csv_dir, exist_ok=True)
        os.makedirs(self.img_dir, exist_ok=True)

        self.get_logger().info(f'分析器啟動！等待資料傳入...')

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
            # 忽略關閉瞬間的轉換錯誤
            pass

    def save_and_plot(self):
        if not self.raw_records:
            self.get_logger().warn('沒有收集到任何數據。')
            return

        self.get_logger().info('正在計算誤差並繪製圖表...')
        df = pd.DataFrame(self.raw_records)
        timestamp = datetime.datetime.now().strftime("%H%M%S")

        # 1. 儲存 CSV
        csv_filename = os.path.join(self.csv_dir, f"hand_data_{timestamp}.csv")
        df.to_csv(csv_filename, index=False)
        self.get_logger().info(f'CSV 已儲存: {csv_filename}')

        # 2. 繪圖 (4x4 網格)
        target_joints = list(range(min(16, len(self.joint_names))))
        fig, axes = plt.subplots(4, 4, figsize=(24, 18))
        axes = axes.flatten()

        for idx, j_idx in enumerate(target_joints):
            name = self.joint_names[j_idx]
            ref = df[f'{name}_ref']
            act = df[f'{name}_act']
            err = df[f'{name}_err']
            
            # 計算最大誤差 (絕對值)
            max_err = err.abs().max()

            # 繪製曲線
            axes[idx].plot(df['time'], ref, 'r--', linewidth=1.5, label='Cmd (Ref)')
            axes[idx].plot(df['time'], act, 'b-', linewidth=1.2, label='Actual (Act)')
            # 繪製誤差曲線 (使用半透明綠色區域或實線)
            axes[idx].plot(df['time'], err, 'g-', alpha=0.6, label='Error')
            
            # 標題顯示最大誤差 (保留四位小數)
            axes[idx].set_title(f'{name}\nMax Err: {max_err:.4f} rad', fontsize=11, fontweight='bold')
            axes[idx].set_xlabel('Time (s)')
            axes[idx].set_ylabel('Pos (rad)')
            axes[idx].grid(True, linestyle=':', alpha=0.6)
            
            if idx == 0:
                axes[idx].legend(loc='upper right', fontsize='small')

        plt.suptitle(f"LeapHand Performance Analysis - {self.today_str}_{timestamp}\n(Red: Cmd, Blue: Actual, Green: Error)", fontsize=20)
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        
        # 儲存圖片
        img_filename = os.path.join(self.img_dir, f"plot_error_{timestamp}.png")
        plt.savefig(img_filename, dpi=120)
        self.get_logger().info(f'分析圖表已儲存: {img_filename}')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = HandTimeDomainAnalyzer()
    
    try:
        # 使用 spin 而不是手動循環
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # 捕捉 Ctrl+C
        node.get_logger().info('正在停止監聽並產出報告...')
    finally:
        # 先執行存檔，再關閉節點
        node.save_and_plot()
        node.destroy_node()
        # 檢查是否已經 shutdown 以避免二次錯誤
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()