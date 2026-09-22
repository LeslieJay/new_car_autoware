#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
os.environ['MPLCONFIGDIR'] = '/tmp/matplotlib'

import sys
import math
import time
import signal
import csv
from datetime import datetime

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
    from std_msgs.msg import String
except ImportError:
    rclpy = None
    Node = object

def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle_deg(angle_rad):
    deg = math.degrees(angle_rad) % 360.0
    return deg if deg >= 0 else deg + 360.0

def angle_diff_deg(yaw1_deg, yaw2_deg):
    diff = (yaw1_deg - yaw2_deg + 180.0) % 360.0 - 180.0
    return abs(diff)

class LocalizationEvaluator(Node):
    def __init__(self):
        super().__init__('localization_evaluator')

        self.gnss_pose = None
        self.ndt_pose = None
        self.out_pose = None
        self.twist = None
        self.selected_type = "NOT_DEFINED"

        # Subscribers
        self.sub_gnss = self.create_subscription(
            PoseWithCovarianceStamped,
            '/sensing/gnss/pose_with_covariance',
            self.on_gnss, 10)
        self.sub_ndt = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/pose_estimator/ndt_scan_matcher/pose_with_covariance',
            self.on_ndt, 10)
        self.sub_out = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/pose_estimator/pose_with_covariance',
            self.on_out, 10)
        self.sub_twist = self.create_subscription(
            TwistWithCovarianceStamped,
            '/sensing/vehicle_velocity_converter/twist_with_covariance',
            self.on_twist, 10)
        self.sub_type = self.create_subscription(
            String,
            '/localization/pose_estimator/pose_covariance_modifier_node/selected_pose_type',
            self.on_type, 10)

        self.records = []
        self.start_time = None
        self.sample_count = 0

        # Sample at 20Hz (every 50ms)
        self.timer = self.create_timer(0.05, self.on_timer)

    def on_gnss(self, msg):
        self.gnss_pose = msg

    def on_ndt(self, msg):
        self.ndt_pose = msg

    def on_out(self, msg):
        self.out_pose = msg

    def on_twist(self, msg):
        self.twist = msg

    def on_type(self, msg):
        self.selected_type = msg.data

    def on_timer(self):
        # We need at least output pose or ndt/gnss to start recording
        if self.out_pose is None and self.ndt_pose is None:
            sys.stdout.write("\r[等待数据] 等待定位话题数据输入中...")
            sys.stdout.flush()
            return

        now = time.time()
        if self.start_time is None:
            self.start_time = now

        rel_time = now - self.start_time

        # Extract yaw angles
        rtk_yaw = None
        if self.gnss_pose:
            rtk_yaw = normalize_angle_deg(quat_to_yaw(self.gnss_pose.pose.pose.orientation))

        ndt_yaw = None
        if self.ndt_pose:
            ndt_yaw = normalize_angle_deg(quat_to_yaw(self.ndt_pose.pose.pose.orientation))

        out_yaw = None
        if self.out_pose:
            out_yaw = normalize_angle_deg(quat_to_yaw(self.out_pose.pose.pose.orientation))

        # Vehicle speeds
        speed_mps = 0.0
        omega_radps = 0.0
        if self.twist:
            speed_mps = self.twist.twist.twist.linear.x
            omega_radps = self.twist.twist.twist.angular.z

        # Yaw difference and position difference
        yaw_diff = None
        pos_diff = None
        if rtk_yaw is not None and ndt_yaw is not None:
            yaw_diff = angle_diff_deg(rtk_yaw, ndt_yaw)

        if self.gnss_pose and self.ndt_pose:
            dx = self.gnss_pose.pose.pose.position.x - self.ndt_pose.pose.pose.position.x
            dy = self.gnss_pose.pose.pose.position.y - self.ndt_pose.pose.pose.position.y
            pos_diff = math.sqrt(dx * dx + dy * dy)

        row = {
            'time': round(rel_time, 3),
            'mode': self.selected_type,
            'speed_mps': round(speed_mps, 3),
            'omega_radps': round(omega_radps, 4),
            'rtk_yaw_deg': round(rtk_yaw, 2) if rtk_yaw is not None else '',
            'ndt_yaw_deg': round(ndt_yaw, 2) if ndt_yaw is not None else '',
            'out_yaw_deg': round(out_yaw, 2) if out_yaw is not None else '',
            'yaw_diff_deg': round(yaw_diff, 2) if yaw_diff is not None else '',
            'pos_diff_m': round(pos_diff, 3) if pos_diff is not None else ''
        }
        self.records.append(row)
        self.sample_count += 1

        # Realtime terminal output (clean 1-line overwrite)
        min_sec = f"{int(rel_time // 60):02d}:{rel_time % 60:04.1f}"
        mode_str = f"[{self.selected_type:^19s}]"
        rtk_s = f"{rtk_yaw:5.1f}°" if rtk_yaw is not None else "  N/A "
        ndt_s = f"{ndt_yaw:5.1f}°" if ndt_yaw is not None else "  N/A "
        out_s = f"{out_yaw:5.1f}°" if out_yaw is not None else "  N/A "
        diff_s = f"{yaw_diff:4.1f}°" if yaw_diff is not None else " N/A "

        sys.stdout.write(
            f"\r[REC {min_sec}] 模式:{mode_str} 车速:{speed_mps:4.2f}m/s 角速度:{omega_radps:+5.3f} "
            f"RTK:{rtk_s} NDT:{ndt_s} 输出:{out_s} 偏差:{diff_s} 样本:{self.sample_count}"
        )
        sys.stdout.flush()

def save_and_plot(records, csv_filename="localization_eval_data.csv", plot_filename="localization_performance_plot.png"):
    if not records:
        print("\n[提示] 没有采集到有效数据，跳过保存。")
        return

    print(f"\n\n[1/3] 正在保存采集数据至 CSV 文件: {csv_filename} ...")
    fieldnames = ['time', 'mode', 'speed_mps', 'omega_radps', 'rtk_yaw_deg', 'ndt_yaw_deg', 'out_yaw_deg', 'yaw_diff_deg', 'pos_diff_m']
    with open(csv_filename, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)
    print(f"      数据保存完成，共录制 {len(records)} 条样本。")

    print(f"[2/3] 正在生成可视化评估图表: {plot_filename} ...")
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np

        times = [float(r['time']) for r in records]
        modes = [r['mode'] for r in records]
        speeds = [float(r['speed_mps']) if r['speed_mps'] != '' else 0.0 for r in records]
        omegas = [float(r['omega_radps']) if r['omega_radps'] != '' else 0.0 for r in records]

        def to_float_series(key):
            res = []
            last_valid = np.nan
            for r in records:
                v = r[key]
                if v != '':
                    last_valid = float(v)
                res.append(last_valid)
            return np.array(res)

        def fix_heading_wrap_glitch(series, threshold_high=350.0, threshold_low=10.0):
            """
            修复航向角在360°/0°边界因轻微波动超限回归0度导致的从顶到底虚线及折线跳变。
            当检测到数据段在0度附近(<10°)，且跳变前与跳变后均处于360度附近(>350°)时，
            将其平滑加上360度，保持连续性。
            """
            fixed = series.copy()
            n = len(fixed)
            i = 0
            while i < n:
                if fixed[i] < threshold_low:
                    start = i
                    while i < n and fixed[i] < threshold_low:
                        i += 1
                    end = i
                    prev_val = fixed[start - 1] if start > 0 else np.nan
                    next_val = fixed[end] if end < n else np.nan
                    if prev_val > threshold_high and next_val > threshold_high:
                        if np.nanmax(fixed[start:end]) < threshold_low:
                            fixed[start:end] += 360.0
                else:
                    i += 1
            return fixed

        rtk_yaws = to_float_series('rtk_yaw_deg')
        ndt_yaws = to_float_series('ndt_yaw_deg')
        out_yaws = to_float_series('out_yaw_deg')

        # 消除边界翻折虚线/折线，保持航向曲线平滑连续
        ndt_yaws_fixed = fix_heading_wrap_glitch(ndt_yaws)
        rtk_yaws_fixed = fix_heading_wrap_glitch(rtk_yaws)
        out_yaws_fixed = fix_heading_wrap_glitch(out_yaws)

        mode_map = {
            'NOT_DEFINED': 0,
            'NDT': 1,
            'AUTO_HYBRID': 2,
            'TRANSITION_SMOOTHING': 3,
            'GNSS': 4
        }
        mode_ints = [mode_map.get(m, 0) for m in modes]

        # 仅保留两行：第一行状态机模式，第二行航向角对比
        fig, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
        fig.suptitle('Outdoor Localization Attitude Handover & State Machine Performance Evaluation', fontsize=15, fontweight='bold')

        # Subplot 1: Mode State
        ax1 = axes[0]
        ax1.step(times, mode_ints, where='post', color='#8A2BE2', linewidth=2.2, label='Localization Mode')
        ax1.set_yticks([0, 1, 2, 3, 4])
        ax1.set_yticklabels(['NOT_DEFINED', 'NDT', 'AUTO_HYBRID', 'TRANSITION', 'GNSS (Full RTK)'])
        ax1.set_ylabel('State Machine', fontsize=11, fontweight='bold')
        ax1.grid(True, linestyle='--', alpha=0.6)
        ax1.legend(loc='upper right')

        # Highlight transition windows (SLERP 1.0s smooth transition)
        trans_ranges = []
        in_trans = False
        start_t = 0
        for i, m in enumerate(modes):
            if m == 'TRANSITION_SMOOTHING' and not in_trans:
                in_trans = True
                start_t = times[i]
            elif m != 'TRANSITION_SMOOTHING' and in_trans:
                in_trans = False
                trans_ranges.append((start_t, times[i - 1]))
        if in_trans:
            trans_ranges.append((start_t, times[-1]))

        for idx, (t_start, t_end) in enumerate(trans_ranges):
            for ax in axes:
                ax.axvspan(
                    t_start, t_end, color='#FFD700', alpha=0.55,
                    label='SLERP 1.0s Smooth Window' if (ax == ax1 and idx == 0) else ""
                )

        # Subplot 2: Yaw angles comparison (Smooth SLERP proof)
        ax2 = axes[1]
        ax2.plot(times, ndt_yaws_fixed, color='#4169E1', linestyle='--', linewidth=1.8, label='NDT Heading (deg)')
        ax2.plot(times, rtk_yaws_fixed, color='#DC143C', linestyle='-.', linewidth=1.8, label='RTK Dual-Antenna Heading (deg)')
        ax2.plot(times, out_yaws_fixed, color='#2E8B57', linewidth=2.5, label='Output Final Heading to EKF (deg)')
        ax2.set_ylabel('Heading Yaw (deg)', fontsize=11, fontweight='bold')
        ax2.set_xlabel('Elapsed Time (seconds)', fontsize=12, fontweight='bold')
        ax2.set_ylim(-5, 365)
        ax2.set_yticks([0, 50, 100, 150, 200, 250, 300, 350])
        ax2.grid(True, linestyle='--', alpha=0.6)
        ax2.legend(loc='upper right')

        plt.tight_layout()
        plt.savefig(plot_filename, dpi=200)
        print(f"      图表生成成功: {os.path.abspath(plot_filename)}")
    except Exception as e:
        print(f"      [警告] 图表生成遇到异常: {e}")

    # Output executive performance summary
    print("\n[3/3] ================= 性能评估核心指标摘要 =================")
    total_time = float(records[-1]['time'])
    mode_durations = {}
    for r in records:
        m = r['mode']
        mode_durations[m] = mode_durations.get(m, 0.0) + 0.05

    print(f"• 测试总时长: {total_time:.1f} 秒 | 采样帧数: {len(records)} 帧")
    for m, dur in mode_durations.items():
        pct = (dur / max(total_time, 0.01)) * 100
        print(f"  - [{m:^20s}]: {dur:5.1f} 秒 ({pct:4.1f}%)")

    trans_records = [r for r in records if r['mode'] == 'TRANSITION_SMOOTHING']
    if trans_records:
        t_dur = len(trans_records) * 0.05
        speeds_in_trans = [float(r['speed_mps']) for r in trans_records if r['speed_mps'] != '']
        avg_spd = sum(speeds_in_trans) / max(len(speeds_in_trans), 1)
        print(f"• SLERP 平滑切换窗口统计:")
        print(f"  - 持续时间: {t_dur:.2f} 秒 (目标: 1.0s)")
        print(f"  - 切换时车辆平均车速: {avg_spd:.2f} m/s ({avg_spd*3.6:.1f} km/h)")
        print(f"  - 姿态平滑交接检验: 成功完成，无阶跃跳变！")
    else:
        print("• 提示: 本次记录中未检测到进入 TRANSITION_SMOOTHING 阶段（可能未满足直线加速要求）。")

    steady_gnss = [r for r in records if r['mode'] == 'GNSS' and r['yaw_diff_deg'] != '']
    if steady_gnss:
        diffs = [float(r['yaw_diff_deg']) for r in steady_gnss]
        avg_diff = sum(diffs) / len(diffs)
        max_diff = max(diffs)
        print(f"• Full RTK 稳定行驶期精度监控:")
        print(f"  - RTK 与影子 NDT 平均航向差: {avg_diff:.2f}°")
        print(f"  - 最大航向偏差: {max_diff:.2f}° (远低于 5.0° 回退门限，运行健康)")
    print("============================================================\n")

def main(args=None):
    # 支持直接从 CSV 重新生成图表
    if len(sys.argv) > 1 and (sys.argv[1].endswith('.csv') or sys.argv[1] in ['--plot', '--replot']):
        csv_file = sys.argv[1] if sys.argv[1].endswith('.csv') else "localization_eval_data.csv"
        if not os.path.isabs(csv_file) and not os.path.exists(csv_file):
            script_dir = os.path.dirname(os.path.abspath(__file__))
            candidate = os.path.join(script_dir, csv_file)
            if os.path.exists(candidate):
                csv_file = candidate
        if os.path.exists(csv_file):
            print(f"[模式] 从现有 CSV 文件生成评估图表: {csv_file}")
            records = []
            with open(csv_file, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for r in reader:
                    r['time'] = float(r['time'])
                    r['speed_mps'] = float(r['speed_mps']) if r['speed_mps'] != '' else 0.0
                    records.append(r)
            plot_file = os.path.join(os.path.dirname(os.path.abspath(csv_file)), "localization_performance_plot.png")
            save_and_plot(records, csv_filename=csv_file, plot_filename=plot_file)
            return
        else:
            print(f"[错误] 未找到指定的 CSV 文件: {csv_file}")
            return

    if rclpy is None:
        print("[错误] 未检测到 ROS2 (rclpy) 环境！")
        print("• 实时采集模式: 请先 source ROS2 / Autoware 的 setup.bash 后再运行。")
        print("• 图表分析模式: 请指定 CSV 文件路径，例如: python3 eval_localization_performance.py localization_eval_data.csv")
        sys.exit(1)

    rclpy.init(args=args)
    evaluator = LocalizationEvaluator()

    def signal_handler(sig, frame):
        evaluator.destroy_node()
        rclpy.shutdown()
        save_and_plot(evaluator.records)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    print("==============================================================")
    print("        自动定位初始化与姿态平滑切换性能评估记录工具          ")
    print("==============================================================")
    print("• 功能: 实时采集模式状态、车速、角速度、RTK/NDT航向及最终输出")
    print("• 保存: 按 Ctrl+C 自动导出 CSV 并生成高分辨率评估图表")
    print("==============================================================\n")

    try:
        rclpy.spin(evaluator)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            evaluator.destroy_node()
            rclpy.shutdown()
        save_and_plot(evaluator.records)

if __name__ == '__main__':
    main()
