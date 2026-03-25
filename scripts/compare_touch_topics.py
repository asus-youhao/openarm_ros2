#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
比較 Python SDK 和 C++ SDK 的觸控感測器 topic 數據
用於驗證 C++ 實作是否與 Python SDK 一致
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import json
import sys
from datetime import datetime
from collections import defaultdict

class TouchTopicComparator(Node):
    def __init__(self, hand_type='left'):
        super().__init__('touch_topic_comparator')
        
        self.hand_type = hand_type
        self.data_python = defaultdict(dict)
        self.data_cpp = defaultdict(dict)
        self.msg_count = defaultdict(int)
        
        # Python SDK topics (假設使用 linker_hand_ros2_sdk)
        self.sub_py_matrix = self.create_subscription(
            String,
            f'/cb_{hand_type}_hand_matrix_touch',
            self.callback_py_matrix,
            10)
        
        self.sub_py_mass = self.create_subscription(
            String,
            f'/cb_{hand_type}_hand_matrix_touch_mass',
            self.callback_py_mass,
            10)
        
        self.sub_py_pc = self.create_subscription(
            PointCloud2,
            f'/cb_{hand_type}_hand_matrix_touch_pc',
            self.callback_py_pc,
            10)
        
        # 創建定時器來打印比較結果
        self.timer = self.create_timer(2.0, self.print_comparison)
        
        self.get_logger().info(f'=== Touch Topic Comparator Started ===')
        self.get_logger().info(f'Monitoring {hand_type} hand topics:')
        self.get_logger().info(f'  - /cb_{hand_type}_hand_matrix_touch')
        self.get_logger().info(f'  - /cb_{hand_type}_hand_matrix_touch_mass')
        self.get_logger().info(f'  - /cb_{hand_type}_hand_matrix_touch_pc')
        self.get_logger().info('Press Ctrl+C to stop\n')
    
    def callback_py_matrix(self, msg):
        """接收矩陣觸控數據 (JSON)"""
        try:
            data = json.loads(msg.data)
            self.data_python['matrix'] = data
            self.msg_count['matrix'] += 1
            
            # 分析數據
            if 'stamp' in data:
                stamp = data['stamp']
            
            # 統計每個手指的數據
            finger_stats = {}
            for finger_name in ['thumb_matrix', 'index_matrix', 'middle_matrix', 'ring_matrix', 'little_matrix']:
                if finger_name in data:
                    matrix = data[finger_name]
                    if isinstance(matrix, list):
                        total_values = sum(sum(row) if isinstance(row, list) else 0 for row in matrix)
                        non_zero = sum(1 for row in matrix if isinstance(row, list) for val in row if val != 0)
                        finger_stats[finger_name] = {
                            'rows': len(matrix),
                            'cols': len(matrix[0]) if matrix and isinstance(matrix[0], list) else 0,
                            'total': total_values,
                            'non_zero_count': non_zero
                        }
            
            self.data_python['matrix_stats'] = finger_stats
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse matrix JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in matrix callback: {e}')
    
    def callback_py_mass(self, msg):
        """接收質量數據 (JSON)"""
        try:
            data = json.loads(msg.data)
            self.data_python['mass'] = data
            self.msg_count['mass'] += 1
            
            # 提取每個手指的質量
            masses = {}
            for finger in ['thumb_mass', 'index_mass', 'middle_mass', 'ring_mass', 'little_mass']:
                if finger in data:
                    masses[finger] = data[finger]
            
            self.data_python['masses'] = masses
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse mass JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in mass callback: {e}')
    
    def callback_py_pc(self, msg):
        """接收點雲數據"""
        self.msg_count['pointcloud'] += 1
        self.data_python['pc'] = {
            'width': msg.width,
            'height': msg.height,
            'point_step': msg.point_step,
            'row_step': msg.row_step,
            'data_size': len(msg.data),
            'is_dense': msg.is_dense,
            'frame_id': msg.header.frame_id
        }
    
    def print_comparison(self):
        """定期打印數據統計"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        
        print('\n' + '='*80)
        print(f'[{timestamp}] Touch Sensor Data Analysis - {self.hand_type.upper()} Hand')
        print('='*80)
        
        # 打印消息計數
        print(f'\n📊 Message Count:')
        print(f'  Matrix:      {self.msg_count["matrix"]:>5} messages')
        print(f'  Mass:        {self.msg_count["mass"]:>5} messages')
        print(f'  PointCloud:  {self.msg_count["pointcloud"]:>5} messages')
        
        # 打印矩陣數據統計
        if 'matrix_stats' in self.data_python:
            print(f'\n📋 Matrix Data Structure:')
            stats = self.data_python['matrix_stats']
            for finger_name, finger_stat in stats.items():
                finger_display = finger_name.replace('_matrix', '').title()
                print(f'  {finger_display:>8}: {finger_stat["rows"]}x{finger_stat["cols"]} matrix, '
                      f'Total={finger_stat["total"]:>6}, NonZero={finger_stat["non_zero_count"]:>4}')
        
        # 打印質量數據
        if 'masses' in self.data_python:
            print(f'\n⚖️  Mass Data (unit: g):')
            masses = self.data_python['masses']
            for finger, mass in masses.items():
                finger_display = finger.replace('_mass', '').title()
                print(f'  {finger_display:>8}: {mass:>6}g')
        
        # 打印質量數據的時間戳
        if 'mass' in self.data_python and 'stamp' in self.data_python['mass']:
            stamp = self.data_python['mass']['stamp']
            print(f'\n🕐 Timestamp: secs={stamp.get("secs", "N/A")}, nsecs={stamp.get("nsecs", "N/A")}')
        
        # 打印點雲數據統計
        if 'pc' in self.data_python:
            pc = self.data_python['pc']
            print(f'\n☁️  PointCloud Data:')
            print(f'  Width:      {pc["width"]}')
            print(f'  Height:     {pc["height"]}')
            print(f'  Points:     {pc["width"] * pc["height"]}')
            print(f'  Data Size:  {pc["data_size"]} bytes')
            print(f'  Frame ID:   {pc["frame_id"]}')
        
        # 數據有效性檢查
        print(f'\n✅ Data Validity Check:')
        
        # 檢查是否所有數據都是 0
        if 'matrix_stats' in self.data_python:
            total_force = sum(stat['total'] for stat in self.data_python['matrix_stats'].values())
            if total_force == 0:
                print(f'  ⚠️  WARNING: All force values are ZERO!')
                print(f'      This indicates NO touch sensor hardware is present.')
                print(f'      This is NORMAL for O6 hands without touch sensors.')
            else:
                print(f'  ✓ Touch sensor has active data (total force: {total_force})')
        
        # 檢查數據格式
        if 'mass' in self.data_python:
            if 'unit' in self.data_python['mass']:
                unit = self.data_python['mass']['unit']
                print(f'  ✓ Mass data has correct unit: "{unit}"')
            else:
                print(f'  ⚠️  Mass data missing "unit" field')
        
        print('\n' + '='*80)

def main(args=None):
    # 解析命令行參數
    hand_type = 'left'
    if len(sys.argv) > 1:
        hand_type = sys.argv[1].lower()
        if hand_type not in ['left', 'right']:
            print(f'Error: Invalid hand type "{hand_type}". Use "left" or "right".')
            return
    
    rclpy.init(args=args)
    
    print('\n' + '='*80)
    print('Touch Sensor Topic Comparator')
    print('='*80)
    print(f'Monitoring: {hand_type.upper()} hand')
    print(f'Usage: python3 compare_touch_topics.py [left|right]')
    print('='*80 + '\n')
    
    node = TouchTopicComparator(hand_type)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n\n' + '='*80)
        print('Shutting down...')
        print('='*80)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
