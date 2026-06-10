#!/usr/bin/env python3
# Copyright (c) 2026 Leaf. D-Robotics.
# SPDX-License-Identifier: MIT
"""
IMU Timestamp Stability Test (Stress Test)

- 订阅 IMU 话题，等待收到第一条消息后再开始压力测试。
- 使用 stress-ng 逐步增加 CPU/内存压力，分阶段记录消息间隔。
- 输出散点图：横轴为消息序列号，纵轴为相邻消息时间戳间隔（秒）。

用法:
    python3 imu_stress_test.py --topic /imu/data --odr 400 --phase-duration 5

依赖:
    rclpy, sensor_msgs, matplotlib, numpy, stress-ng (系统工具)
"""

import os
import sys
import subprocess
import signal
import time
import argparse
from collections import namedtuple

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Imu

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# ==================== 压力阶段定义 ====================
def build_phases(cpu_cores, mem_total_mb, phase_duration=5):
    """
    构造加压阶段列表，每个阶段包含:
        duration : 持续时间(秒)
        label    : 阶段描述
        cmd      : stress-ng 命令列表 (None 表示无压力)
    """
    phases = [
        {'duration': phase_duration, 'label': 'Idle (no stress)', 'cmd': None},
    ]
    # 逐步增加 CPU 核心数
    for n in range(1, cpu_cores + 1):
        phases.append({
            'duration': phase_duration,
            'label': f'CPU stress ({n} core{"s" if n > 1 else ""})',
            'cmd': ['stress-ng', '--cpu', str(n), '--timeout', f'{phase_duration}s']
        })
    # CPU 全开 + 内存逐步增加
    for ratio in [0.2, 0.4, 0.6, 0.8, 1.0]:
        mem_mb = max(int(mem_total_mb * ratio), 256)
        phases.append({
            'duration': phase_duration,
            'label': f'CPU all + Memory {ratio*100:.0f}% ({mem_mb} MB)',
            'cmd': ['stress-ng', '--cpu', str(cpu_cores),
                    '--vm', '1', '--vm-bytes', f'{mem_mb}M',
                    '--timeout', f'{phase_duration}s']
        })
    # 满压力阶段 (稍长)
    phases.append({
        'duration': phase_duration * 2,
        'label': 'Full stress (CPU all + max memory)',
        'cmd': ['stress-ng', '--cpu', str(cpu_cores),
                '--vm', '2', '--vm-bytes', f'{int(mem_total_mb * 0.9)}M',
                '--timeout', f'{phase_duration * 2}s']
    })
    # 恢复空闲
    phases.append({
        'duration': phase_duration,
        'label': 'Recovery (no stress)',
        'cmd': None
    })
    return phases


# ==================== ROS 2 节点 ====================
class ImuStressNode(Node):
    def __init__(self, topic, odr, cpu_cores, mem_total_mb, phase_duration):
        super().__init__('imu_stress_node')
        self.topic = topic
        self.expected_interval = 1.0 / odr if odr > 0 else 0.0

        # 数据记录： (序列号, 间隔, 阶段索引)
        self.records = []
        self.prev_time = None
        self.seq = 0
        self.current_phase_idx = 0

        # 状态控制
        self.data_ready = False         # 收到第一条消息后置 True
        self.phases = build_phases(cpu_cores, mem_total_mb, phase_duration)
        self.stress_proc = None
        self.start_time = None          # 本阶段开始时刻 (rclcpp Time)
        self.phase_start_time = None    # 同上

        # 订阅 IMU
        self.sub = self.create_subscription(Imu, topic, self.callback, 10)
        self.get_logger().info(f'Waiting for first message on {topic} ...')

    def callback(self, msg):
        # 提取秒数时间戳
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self.data_ready:
            self.data_ready = True
            self.get_logger().info(f'First message received (t={t:.3f}), data collection begins.')
            # 第一次收到时，不计算间隔，仅记录起始时间
            self.prev_time = t
            self.start_time = self.get_clock().now()
            return

        # 正常计算间隔
        interval = t - self.prev_time
        self.records.append((self.seq, interval, self.current_phase_idx))
        self.prev_time = t
        self.seq += 1

    def start_stress(self, phase):
        """启动一个压力阶段的 stress-ng 进程"""
        cmd = phase['cmd']
        if cmd:
            self.get_logger().info(f'Starting: {phase["label"]}')
            self.stress_proc = subprocess.Popen(cmd)
        else:
            self.get_logger().info(f'Phase: {phase["label"]} (no stress)')
            self.stress_proc = None

    def stop_stress(self):
        """终止当前压力进程"""
        if self.stress_proc is not None:
            self.get_logger().info('Stopping stress process...')
            self.stress_proc.terminate()
            try:
                self.stress_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.stress_proc.kill()
                self.stress_proc.wait()
            self.stress_proc = None

    def run_experiment(self):
        """
        主实验循环:
        1. 等待 data_ready (收到第一条消息) – 通过 spin_once 实现。
        2. 收到后，进入阶段循环，每个阶段 spin 直到阶段结束。
        """
        # 等待第一条消息（超时 30 秒）
        timeout = 30
        waited = 0
        while rclpy.ok() and not self.data_ready:
            rclpy.spin_once(self, timeout_sec=0.1)
            waited += 0.1
            if waited > timeout:
                self.get_logger().error('Timeout waiting for IMU data.')
                return

        # 记录实验真正开始的时刻（此时已经有不少数据被收集）
        self.get_logger().info('Beginning stress test phases.')
        total_phases = len(self.phases)

        for phase_idx, phase in enumerate(self.phases):
            self.current_phase_idx = phase_idx
            self.start_stress(phase)
            phase_start = self.get_clock().now()
            duration_ns = phase['duration'] * 1e9

            # 在当前阶段内持续 spin 收集数据
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                elapsed = (self.get_clock().now() - phase_start).nanoseconds
                if elapsed >= duration_ns:
                    break

            self.stop_stress()

        self.get_logger().info('Experiment finished.')


# ==================== 绘图 ====================
def plot_results(records, phases, expected_interval, output_file):
    if not records:
        print("No data collected – nothing to plot.")
        return

    seqs = [r[0] for r in records]
    intervals = [r[1] for r in records]
    phase_ids = [r[2] for r in records]

    # 准备颜色
    unique_phases = sorted(set(phase_ids))
    cmap = plt.cm.get_cmap('tab10', len(unique_phases))
    colors = {p: cmap(i) for i, p in enumerate(unique_phases)}

    plt.style.use('seaborn-v0_8-whitegrid')
    fig, ax = plt.subplots(figsize=(12, 6))

    # 按阶段分别绘制散点
    for p in unique_phases:
        mask = [ph == p for ph in phase_ids]
        ax.scatter(np.array(seqs)[mask], np.array(intervals)[mask],
                   c=[colors[p]], label=phases[p]['label'],
                   s=3, alpha=0.7, edgecolors='none')

    # 期望间隔线
    if expected_interval > 0:
        ax.axhline(y=expected_interval, color='gray', linestyle='--', linewidth=1,
                   label=f'Expected: {expected_interval*1000:.2f} ms')

    # 阶段边界垂直线
    boundaries = []
    current = phase_ids[0]
    for i, p in enumerate(phase_ids):
        if p != current:
            boundaries.append(seqs[i])
            current = p
    for b in boundaries:
        ax.axvline(x=b, color='black', linestyle=':', linewidth=0.8, alpha=0.5)

    ax.set_xlabel('Sequence Number', fontsize=12)
    ax.set_ylabel('Timestamp Interval (s)', fontsize=12)
    ax.set_title('IMU Timestamp Stability Under Progressive Stress', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=9, markerscale=2)
    ax.tick_params(labelsize=10)

    plt.tight_layout()
    plt.savefig(output_file, dpi=300)
    print(f'Plot saved to {output_file}')
    plt.close()


# ==================== 辅助函数 ====================
def get_cpu_cores():
    return os.cpu_count() or 4

def get_total_memory_mb():
    try:
        with open('/proc/meminfo', 'r') as f:
            for line in f:
                if line.startswith('MemTotal:'):
                    return int(line.split()[1]) // 1024
    except:
        pass
    return 2048  # 默认 2GB

def main():
    parser = argparse.ArgumentParser(description='IMU Timestamp Stability Under Stress')
    parser.add_argument('--topic', default='/imu/data', help='IMU topic name')
    parser.add_argument('--odr', type=float, default=400.0, help='Expected output data rate (Hz)')
    parser.add_argument('--phase-duration', type=float, default=5.0, help='Duration of each stress phase (seconds)')
    parser.add_argument('--output', default='imu_timestamp_stability.pdf', help='Output plot file')
    args = parser.parse_args()

    cpu_cores = get_cpu_cores()
    mem_total_mb = get_total_memory_mb()
    print(f'System: {cpu_cores} CPU cores, {mem_total_mb} MB memory.')
    print(f'Each stress phase lasts {args.phase_duration} seconds.')

    rclpy.init(args=sys.argv)

    node = ImuStressNode(args.topic, args.odr, cpu_cores, mem_total_mb, args.phase_duration)

    try:
        node.run_experiment()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        # 绘制结果
        plot_results(node.records, node.phases, node.expected_interval, args.output)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()