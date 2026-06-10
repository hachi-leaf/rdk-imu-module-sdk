#!/usr/bin/env python3
"""
纯 Python 版 IMU 时间戳稳定性测试（使用 rdkimu 库 + stress-ng）

- 初始化 IMU（SPI 自动扫描，配置按 RDK_X5_DEFAULT_CONFIG）
- 逐步增加 CPU/内存压力，分阶段记录相邻消息的时间戳间隔
- 输出论文风格散点图：横轴为序列号，纵轴为间隔（秒）

用法：
    python3 imu_stress_test.py --odr 400 --phase-duration 5
依赖：
    rdkimu (Python 库), stress-ng (系统工具), matplotlib, numpy
"""

import os
import sys
import subprocess
import time
import argparse
import threading
from collections import deque

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# 导入 rdkimu
from rdkimu import IMU, RDK_X5_DEFAULT_CONFIG
from rdkimu.imu import RDK_IMU_DEVICE


# ==================== 压力阶段定义 ====================
def build_phases(cpu_cores, mem_total_mb, phase_duration=5):
    """构造压力阶段列表（与 ROS 版相同）"""
    phases = [
        {'duration': phase_duration, 'label': 'Idle (no stress)', 'cmd': None},
    ]
    # CPU 压力逐步增加
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
    # 满压力（稍长）
    phases.append({
        'duration': phase_duration * 2,
        'label': 'Full stress (CPU all + max memory)',
        'cmd': ['stress-ng', '--cpu', str(cpu_cores),
                '--vm', '2', '--vm-bytes', f'{int(mem_total_mb * 0.9)}M',
                '--timeout', f'{phase_duration * 2}s']
    })
    # 恢复阶段
    phases.append({
        'duration': phase_duration,
        'label': 'Recovery (no stress)',
        'cmd': None
    })
    return phases


# ==================== 数据记录 ====================
class DataCollector:
    """线程安全的数据收集器"""
    def __init__(self):
        self.records = []          # (seq, interval_ns, phase_idx)
        self.prev_ts = None
        self.seq = 0
        self.current_phase = 0
        self.lock = threading.Lock()

    def add(self, timestamp_ns, phase_idx):
        with self.lock:
            if self.prev_ts is None:
                self.prev_ts = timestamp_ns
                return
            interval = timestamp_ns - self.prev_ts
            self.records.append((self.seq, interval, phase_idx))
            self.prev_ts = timestamp_ns
            self.seq += 1

    def get_records(self):
        with self.lock:
            return list(self.records)


# ==================== 主测试逻辑 ====================
def run_stress_test(odr, phase_duration):
    cpu_cores = os.cpu_count() or 4
    mem_total_mb = get_total_memory_mb()
    print(f"System: {cpu_cores} cores, {mem_total_mb} MB RAM")
    print(f"Expected ODR: {odr} Hz, Phase duration: {phase_duration}s")

    # 初始化 IMU
    imu = IMU()
    try:
        imu.bus(spi_accel_speed=1000000, spi_gyro_speed=1000000)
        imu.config(RDK_X5_DEFAULT_CONFIG)
        imu.enable()
        print("IMU enabled, waiting for first data...")
    except Exception as e:
        print(f"IMU initialization failed: {e}")
        sys.exit(1)

    collector = DataCollector()
    phases = build_phases(cpu_cores, mem_total_mb, phase_duration)

    # 先读取第一帧，避免 prev_ts 为 None
    first_data = imu.read_fused(fuse_by=RDK_IMU_DEVICE.ACCEL, max_age_ns=50000000)
    first_ts = first_data.accel.timestamp_ns
    collector.prev_ts = first_ts
    print(f"First timestamp: {first_ts}")

    stress_proc = None
    try:
        for phase_idx, phase in enumerate(phases):
            collector.current_phase = phase_idx
            # 启动压力（如果有）
            cmd = phase['cmd']
            if cmd:
                print(f"Starting phase: {phase['label']}")
                stress_proc = subprocess.Popen(cmd)
            else:
                print(f"Phase: {phase['label']}")

            phase_start = time.monotonic()
            duration = phase['duration']

            # 持续读取 IMU 数据直到阶段结束
            while True:
                # read_fused 会阻塞直到有数据，所以每次都会得到有效帧
                data = imu.read_fused(fuse_by=RDK_IMU_DEVICE.ACCEL, max_age_ns=50000000)
                ts = data.accel.timestamp_ns
                collector.add(ts, phase_idx)

                # 检查超时
                if time.monotonic() - phase_start >= duration:
                    break

            # 结束当前阶段的压力进程
            if stress_proc is not None:
                stress_proc.terminate()
                try:
                    stress_proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    stress_proc.kill()
                    stress_proc.wait()
                stress_proc = None

        print("Experiment finished.")
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        imu.disable()
        print("IMU disabled.")

    return collector.get_records(), phases, 1.0 / odr if odr > 0 else 0.0


# ==================== 绘图 ====================
def plot_results(records, phases, expected_interval, output_file='imu_timestamp_stability.pdf'):
    if not records:
        print("No data to plot.")
        return

    seqs = [r[0] for r in records]
    intervals_ns = [r[1] for r in records]
    intervals_s = [ns * 1e-9 for ns in intervals_ns]  # 转为秒
    phase_ids = [r[2] for r in records]

    unique_phases = sorted(set(phase_ids))
    cmap = plt.cm.get_cmap('tab10', len(unique_phases))
    colors = {p: cmap(i) for i, p in enumerate(unique_phases)}

    plt.style.use('seaborn-v0_8-whitegrid')
    fig, ax = plt.subplots(figsize=(12, 6))

    # 按阶段绘制散点
    for p in unique_phases:
        mask = [ph == p for ph in phase_ids]
        ax.scatter(np.array(seqs)[mask], np.array(intervals_s)[mask],
                   c=[colors[p]], label=phases[p]['label'],
                   s=3, alpha=0.7, edgecolors='none')

    # 期望间隔线
    if expected_interval > 0:
        ax.axhline(y=expected_interval, color='gray', linestyle='--', linewidth=1,
                   label=f'Expected: {expected_interval*1000:.2f} ms')

    # 阶段边界
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


def get_total_memory_mb():
    try:
        with open('/proc/meminfo', 'r') as f:
            for line in f:
                if line.startswith('MemTotal:'):
                    return int(line.split()[1]) // 1024
    except:
        pass
    return 2048


def main():
    parser = argparse.ArgumentParser(description='Python IMU Timestamp Stability Test')
    parser.add_argument('--odr', type=float, default=400.0, help='Expected output data rate (Hz)')
    parser.add_argument('--phase-duration', type=float, default=5.0, help='Duration of each stress phase (seconds)')
    parser.add_argument('--output', default='imu_timestamp_stability_python.pdf', help='Output plot file')
    args = parser.parse_args()

    records, phases, expected_interval = run_stress_test(args.odr, args.phase_duration)
    plot_results(records, phases, expected_interval, args.output)


if __name__ == '__main__':
    main()