#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record_and_analyze_localization.py
================================================================================
Autoware 多定位源数据记录、稳定性监控与综合诊断分析工具

功能概述：
1. 实时多源数据采集：
   - RTK (GNSS) 位姿、协方差、RTK定位状态 (Fixed/Float/DGPS/Single/Lost)
   - NDT 激光配准位姿、协方差、变换概率得分 (Score/Transform Probability)、
     最近体素似然、迭代次数 (iteration_num)、配准耗时 (exe_time_ms)
   - EKF 最终融合输出位姿、线速度、角速度 (kinematic_state)
   - 定位融合节点当前生效模式 (selected_pose_type: NDT/FULL_GNSS/AUTO_HYBRID/TRANSITION)
2. 实时多源差距与稳定性评估：
   - RTK vs NDT 空间距离偏差 (m) 与航向角度偏差 (deg)
   - EKF vs RTK / EKF vs NDT 跟踪偏差
   - 帧间位姿突变/跳变检测（位置突跳、航向阶跃）
   - NDT 低分告警与 RTK 降级丢星事件实时记录
3. 产出物（测试结束或按 Ctrl+C 自动生成）：
   - localization_raw_data.csv: 完整时序数据表
   - localization_events.csv: 异常跳变与状态切换事件明细表
   - localization_analysis_report.md: 专业工程诊断与统计评估报告
   - localization_performance_analysis.png: 5合1全景可视化分析图表
4. 支持离线回放分析：
   - 可直接加载已录制的 CSV 文件重新分析并生成报告与图表：
     python3 record_and_analyze_localization.py <path_to_csv>
================================================================================
"""

import os
os.environ['MPLCONFIGDIR'] = '/tmp/matplotlib'

import sys
import math
import time
import signal
import csv
import argparse
from datetime import datetime

# ROS2 依赖导入，若离线分析时未安装环境则优雅降级
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import String, Float64
except ImportError:
    rclpy = None
    Node = object

# 自定义/扩展 debug 消息导入处理
Float32Stamped = None
Int32Stamped = None
try:
    from autoware_internal_debug_msgs.msg import Float32Stamped, Int32Stamped
except Exception:
    try:
        from tier4_debug_msgs.msg import Float32Stamped, Int32Stamped
    except Exception:
        Float32Stamped = None
        Int32Stamped = None


def quat_to_yaw(q):
    """从四元数计算偏航角 Yaw (单位: 弧度)"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle_deg(angle_rad):
    """弧度转角度，并归一化到 [0, 360)"""
    deg = math.degrees(angle_rad) % 360.0
    return deg if deg >= 0 else deg + 360.0


def angle_diff_deg(yaw1_deg, yaw2_deg):
    """计算两个角度在最小圆弧方向上的绝对差值 [0, 180] 度"""
    if yaw1_deg is None or yaw2_deg is None:
        return None
    diff = (yaw1_deg - yaw2_deg + 180.0) % 360.0 - 180.0
    return abs(diff)


def extract_pose_stddev(covariance):
    """从 6x6 协方差矩阵中提取 xy 位置标准差和 yaw 标准差"""
    if covariance is None or len(covariance) < 36:
        return 0.0, 0.0
    var_x = max(covariance[0], 0.0)
    var_y = max(covariance[7], 0.0)
    stddev_xy = math.sqrt(var_x + var_y)
    var_yaw = max(covariance[35], 0.0)
    stddev_yaw_deg = math.degrees(math.sqrt(var_yaw))
    return stddev_xy, stddev_yaw_deg


def format_rtk_status(status_code):
    """将 RTK 状态码映射为人性化描述"""
    if status_code is None:
        return "UNKNOWN", -1
    mapping = {
        4: ("FIXED", 4),     # 载波相位固定解 (厘米级)
        5: ("FLOAT", 5),     # 浮点解 (分米级)
        2: ("DGPS", 2),      # 伪距差分 (米级)
        1: ("SINGLE", 1),    # 单点解
        0: ("INVALID", 0),   # 无效解
    }
    return mapping.get(status_code, (f"CODE_{status_code}", status_code))


class LocalizationRecorderNode(Node):
    def __init__(self, sample_rate_hz=20.0, log_root="/home/nvidia/autoware/log/localization_eval"):
        super().__init__('localization_data_recorder')

        self.sample_rate_hz = sample_rate_hz
        self.dt = 1.0 / sample_rate_hz
        self.log_root = log_root

        # 当前缓存的传感器与定位源数据
        self.gnss_pose = None
        self.nav_sat_fix = None
        self.ndt_pose = None
        self.ekf_odom = None
        self.selected_out_pose = None
        self.twist = None
        self.selected_pose_type = "NOT_DEFINED"

        # NDT 质量参数
        self.ndt_score = None
        self.ndt_likelihood = None
        self.iteration_num = None
        self.ndt_exe_time_ms = None

        # 稳定性监控历史数据
        self.prev_ekf_time = None
        self.prev_ekf_x = None
        self.prev_ekf_y = None
        self.prev_ekf_yaw = None

        self.prev_ndt_x = None
        self.prev_ndt_y = None

        self.prev_mode = None
        self.prev_rtk_state = None

        # 统计计数器
        self.jump_events_count = 0
        self.rtk_loss_count = 0
        self.mode_change_count = 0

        self.records = []
        self.events = []
        self.start_wall_time = None
        self.sample_count = 0

        # QoS 设置
        qos_default = 10
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 订阅：RTK 与 GNSS 话题
        self.sub_gnss_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/sensing/gnss/pose_with_covariance',
            self.on_gnss_pose, qos_default)

        self.sub_nav_sat_fix = self.create_subscription(
            NavSatFix,
            '/sensing/gnss/rtk/nav_sat_fix',
            self.on_nav_sat_fix, qos_sensor)

        # 订阅：NDT 话题
        self.sub_ndt_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/pose_estimator/ndt_scan_matcher/pose_with_covariance',
            self.on_ndt_pose, qos_default)

        # 订阅：NDT 匹配得分与耗时 (如消息类型可用)
        if Float32Stamped is not None:
            self.sub_ndt_score = self.create_subscription(
                Float32Stamped,
                '/localization/pose_estimator/ndt_scan_matcher/transform_probability',
                self.on_ndt_score, qos_default)
            self.sub_ndt_likelihood = self.create_subscription(
                Float32Stamped,
                '/localization/pose_estimator/ndt_scan_matcher/nearest_voxel_transformation_likelihood',
                self.on_ndt_likelihood, qos_default)
            self.sub_ndt_exe_time = self.create_subscription(
                Float32Stamped,
                '/localization/pose_estimator/ndt_scan_matcher/exe_time_ms',
                self.on_ndt_exe_time, qos_default)

        if Int32Stamped is not None:
            self.sub_ndt_iter = self.create_subscription(
                Int32Stamped,
                '/localization/pose_estimator/ndt_scan_matcher/iteration_num',
                self.on_ndt_iter, qos_default)

        # 订阅：EKF 与最终融合输出
        self.sub_ekf_odom = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.on_ekf_odom, qos_default)

        self.sub_selected_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/pose_estimator/pose_with_covariance',
            self.on_selected_pose, qos_default)

        # 订阅：底盘车速与角速度
        self.sub_twist = self.create_subscription(
            TwistWithCovarianceStamped,
            '/sensing/vehicle_velocity_converter/twist_with_covariance',
            self.on_twist, qos_default)

        # 订阅：当前活动定位源模式
        self.sub_mode = self.create_subscription(
            String,
            '/localization/pose_estimator/pose_covariance_modifier_node/selected_pose_type',
            self.on_mode, qos_default)

        # 周期性定时采样
        self.timer = self.create_timer(self.dt, self.on_timer)

    # 订阅回调函数
    def on_gnss_pose(self, msg): self.gnss_pose = msg
    def on_nav_sat_fix(self, msg): self.nav_sat_fix = msg
    def on_ndt_pose(self, msg): self.ndt_pose = msg
    def on_ekf_odom(self, msg): self.ekf_odom = msg
    def on_selected_pose(self, msg): self.selected_out_pose = msg
    def on_twist(self, msg): self.twist = msg
    def on_mode(self, msg): self.selected_pose_type = msg.data
    def on_ndt_score(self, msg): self.ndt_score = msg.data
    def on_ndt_likelihood(self, msg): self.ndt_likelihood = msg.data
    def on_ndt_exe_time(self, msg): self.ndt_exe_time_ms = msg.data
    def on_ndt_iter(self, msg): self.iteration_num = msg.data

    def on_timer(self):
        # 只要有任一定位源开始输出，即启动记录
        if self.ekf_odom is None and self.gnss_pose is None and self.ndt_pose is None:
            sys.stdout.write("\r[等待就绪] 等待定位话题数据输出中 (RTK/NDT/EKF)...")
            sys.stdout.flush()
            return

        now = time.time()
        if self.start_wall_time is None:
            self.start_wall_time = now
        rel_time = now - self.start_wall_time

        # 1. 提取 RTK 数据
        rtk_x, rtk_y, rtk_z, rtk_yaw = None, None, None, None
        rtk_std_xy, rtk_std_yaw = None, None
        if self.gnss_pose:
            p = self.gnss_pose.pose.pose.position
            rtk_x, rtk_y, rtk_z = p.x, p.y, p.z
            rtk_yaw = normalize_angle_deg(quat_to_yaw(self.gnss_pose.pose.pose.orientation))
            rtk_std_xy, rtk_std_yaw = extract_pose_stddev(self.gnss_pose.pose.covariance)

        rtk_status_desc, rtk_code = "UNKNOWN", -1
        if self.nav_sat_fix:
            rtk_status_desc, rtk_code = format_rtk_status(self.nav_sat_fix.status.status)

        # 2. 提取 NDT 数据
        ndt_x, ndt_y, ndt_z, ndt_yaw = None, None, None, None
        ndt_std_xy, ndt_std_yaw = None, None
        if self.ndt_pose:
            p = self.ndt_pose.pose.pose.position
            ndt_x, ndt_y, ndt_z = p.x, p.y, p.z
            ndt_yaw = normalize_angle_deg(quat_to_yaw(self.ndt_pose.pose.pose.orientation))
            ndt_std_xy, ndt_std_yaw = extract_pose_stddev(self.ndt_pose.pose.covariance)

        # 3. 提取 EKF 融合数据 (车辆当前实际运动态)
        ekf_x, ekf_y, ekf_z, ekf_yaw = None, None, None, None
        speed_mps = 0.0
        yaw_rate_degps = 0.0
        if self.ekf_odom:
            p = self.ekf_odom.pose.pose.position
            ekf_x, ekf_y, ekf_z = p.x, p.y, p.z
            ekf_yaw = normalize_angle_deg(quat_to_yaw(self.ekf_odom.pose.pose.orientation))
            speed_mps = self.ekf_odom.twist.twist.linear.x
            yaw_rate_degps = math.degrees(self.ekf_odom.twist.twist.angular.z)
        elif self.twist:
            speed_mps = self.twist.twist.twist.linear.x
            yaw_rate_degps = math.degrees(self.twist.twist.twist.angular.z)

        # 4. 提取输出到 EKF 的选定数据
        sel_x, sel_y, sel_yaw = None, None, None
        if self.selected_out_pose:
            p = self.selected_out_pose.pose.pose.position
            sel_x, sel_y = p.x, p.y
            sel_yaw = normalize_angle_deg(quat_to_yaw(self.selected_out_pose.pose.pose.orientation))

        # 5. 分析定位源之间的差距 (Discrepancy)
        rtk_vs_ndt_pos_diff = None
        rtk_vs_ndt_yaw_diff = None
        if rtk_x is not None and ndt_x is not None:
            rtk_vs_ndt_pos_diff = math.sqrt((rtk_x - ndt_x)**2 + (rtk_y - ndt_y)**2)
            rtk_vs_ndt_yaw_diff = angle_diff_deg(rtk_yaw, ndt_yaw)

        ekf_vs_rtk_pos_diff = None
        ekf_vs_rtk_yaw_diff = None
        if ekf_x is not None and rtk_x is not None:
            ekf_vs_rtk_pos_diff = math.sqrt((ekf_x - rtk_x)**2 + (ekf_y - rtk_y)**2)
            ekf_vs_rtk_yaw_diff = angle_diff_deg(ekf_yaw, rtk_yaw)

        ekf_vs_ndt_pos_diff = None
        ekf_vs_ndt_yaw_diff = None
        if ekf_x is not None and ndt_x is not None:
            ekf_vs_ndt_pos_diff = math.sqrt((ekf_x - ndt_x)**2 + (ekf_y - ndt_y)**2)
            ekf_vs_ndt_yaw_diff = angle_diff_deg(ekf_yaw, ndt_yaw)

        # 6. 定位稳定性分析与跳变检测 (Stability Jump Detection)
        ekf_jump_pos = 0.0
        ekf_jump_yaw = 0.0
        jump_anomaly = False
        anomaly_reason = []

        if self.prev_ekf_time is not None and ekf_x is not None and self.prev_ekf_x is not None:
            dt_step = max(now - self.prev_ekf_time, 0.001)
            actual_disp = math.sqrt((ekf_x - self.prev_ekf_x)**2 + (ekf_y - self.prev_ekf_y)**2)
            expected_max_disp = abs(speed_mps) * dt_step + 0.12  # 允许的车速积分上限加冗余
            if actual_disp > expected_max_disp and actual_disp > 0.15:
                ekf_jump_pos = actual_disp - (abs(speed_mps) * dt_step)
                jump_anomaly = True
                anomaly_reason.append(f"EKF位置突变({ekf_jump_pos:.2f}m)")

            actual_yaw_disp = angle_diff_deg(ekf_yaw, self.prev_ekf_yaw)
            expected_max_yaw = abs(yaw_rate_degps) * dt_step + 2.0
            if actual_yaw_disp > expected_max_yaw and actual_yaw_disp > 3.0:
                ekf_jump_yaw = actual_yaw_disp
                jump_anomaly = True
                anomaly_reason.append(f"EKF航向突跃({ekf_jump_yaw:.1f}°)")

        # 检查 NDT 跳变
        if self.prev_ndt_x is not None and ndt_x is not None:
            ndt_disp = math.sqrt((ndt_x - self.prev_ndt_x)**2 + (ndt_y - self.prev_ndt_y)**2)
            if ndt_disp > 0.35:
                jump_anomaly = True
                anomaly_reason.append(f"NDT单帧跳跃({ndt_disp:.2f}m)")

        # 检查 RTK 状态切换与失锁
        if self.prev_rtk_state is not None and rtk_code != self.prev_rtk_state:
            if self.prev_rtk_state == 4 and rtk_code != 4:
                self.rtk_loss_count += 1
                self.events.append({
                    'time': round(rel_time, 3),
                    'type': 'RTK失锁降级',
                    'detail': f"从 Fixed(4) 降级为 {rtk_status_desc}({rtk_code})",
                    'mode': self.selected_pose_type,
                    'speed': round(speed_mps, 2)
                })

        # 检查模式切换事件
        if self.prev_mode is not None and self.selected_pose_type != self.prev_mode:
            self.mode_change_count += 1
            self.events.append({
                'time': round(rel_time, 3),
                'type': '定位模式切换',
                'detail': f"从 {self.prev_mode} 切换为 {self.selected_pose_type}",
                'mode': self.selected_pose_type,
                'speed': round(speed_mps, 2)
            })

        # 记录异常突变事件
        if jump_anomaly:
            self.jump_events_count += 1
            self.events.append({
                'time': round(rel_time, 3),
                'type': '定位跳变异常',
                'detail': "; ".join(anomaly_reason),
                'mode': self.selected_pose_type,
                'speed': round(speed_mps, 2)
            })

        # 更新历史位姿
        self.prev_ekf_time = now
        self.prev_ekf_x, self.prev_ekf_y, self.prev_ekf_yaw = ekf_x, ekf_y, ekf_yaw
        self.prev_ndt_x, self.prev_ndt_y = ndt_x, ndt_y
        self.prev_mode = self.selected_pose_type
        self.prev_rtk_state = rtk_code

        # 整理单帧数据记录
        row = {
            'time': round(rel_time, 3),
            'timestamp': round(now, 4),
            'mode': self.selected_pose_type,
            'rtk_state': rtk_status_desc,
            'rtk_code': rtk_code,
            'speed_mps': round(speed_mps, 3),
            'yaw_rate_degps': round(yaw_rate_degps, 2),
            # RTK
            'rtk_x': round(rtk_x, 3) if rtk_x is not None else '',
            'rtk_y': round(rtk_y, 3) if rtk_y is not None else '',
            'rtk_z': round(rtk_z, 3) if rtk_z is not None else '',
            'rtk_yaw_deg': round(rtk_yaw, 2) if rtk_yaw is not None else '',
            'rtk_std_xy': round(rtk_std_xy, 3) if rtk_std_xy is not None else '',
            'rtk_std_yaw': round(rtk_std_yaw, 2) if rtk_std_yaw is not None else '',
            # NDT
            'ndt_x': round(ndt_x, 3) if ndt_x is not None else '',
            'ndt_y': round(ndt_y, 3) if ndt_y is not None else '',
            'ndt_z': round(ndt_z, 3) if ndt_z is not None else '',
            'ndt_yaw_deg': round(ndt_yaw, 2) if ndt_yaw is not None else '',
            'ndt_std_xy': round(ndt_std_xy, 3) if ndt_std_xy is not None else '',
            # NDT 得分与性能
            'ndt_score': round(self.ndt_score, 4) if self.ndt_score is not None else '',
            'ndt_likelihood': round(self.ndt_likelihood, 4) if self.ndt_likelihood is not None else '',
            'ndt_iter': self.iteration_num if self.iteration_num is not None else '',
            'ndt_exe_time_ms': round(self.ndt_exe_time_ms, 2) if self.ndt_exe_time_ms is not None else '',
            # EKF 最终位姿
            'ekf_x': round(ekf_x, 3) if ekf_x is not None else '',
            'ekf_y': round(ekf_y, 3) if ekf_y is not None else '',
            'ekf_z': round(ekf_z, 3) if ekf_z is not None else '',
            'ekf_yaw_deg': round(ekf_yaw, 2) if ekf_yaw is not None else '',
            # 多源偏差 (Discrepancy)
            'diff_rtk_ndt_pos_m': round(rtk_vs_ndt_pos_diff, 3) if rtk_vs_ndt_pos_diff is not None else '',
            'diff_rtk_ndt_yaw_deg': round(rtk_vs_ndt_yaw_diff, 2) if rtk_vs_ndt_yaw_diff is not None else '',
            'diff_ekf_rtk_pos_m': round(ekf_vs_rtk_pos_diff, 3) if ekf_vs_rtk_pos_diff is not None else '',
            'diff_ekf_ndt_pos_m': round(ekf_vs_ndt_pos_diff, 3) if ekf_vs_ndt_pos_diff is not None else '',
            # 跳变量
            'jump_pos_m': round(ekf_jump_pos, 3),
            'jump_yaw_deg': round(ekf_jump_yaw, 2),
            'is_jump_event': 1 if jump_anomaly else 0
        }
        self.records.append(row)
        self.sample_count += 1

        # 终端单行动态覆盖看板 (清晰专业实时观察)
        min_sec = f"{int(rel_time // 60):02d}:{rel_time % 60:04.1f}"
        mode_str = f"[{self.selected_pose_type:^12s}]"
        rtk_tag = f"{rtk_status_desc:^6s}"
        ndt_score_str = f"{self.ndt_score:.2f}" if self.ndt_score is not None else " N/A"
        pos_diff_str = f"{rtk_vs_ndt_pos_diff:.2f}m" if rtk_vs_ndt_pos_diff is not None else " N/A "
        yaw_diff_str = f"{rtk_vs_ndt_yaw_diff:.1f}°" if rtk_vs_ndt_yaw_diff is not None else " N/A "
        jump_flag_str = "【⚠️跳变】" if jump_anomaly else "       "

        sys.stdout.write(
            f"\r[T+{min_sec}] 模式:{mode_str} RTK:{rtk_tag} 车速:{speed_mps:4.1f}m/s "
            f"NDT分:{ndt_score_str} RTK-NDT距:{pos_diff_str} 偏角:{yaw_diff_str} "
            f"异常:{self.jump_events_count}次 {jump_flag_str}"
        )
        sys.stdout.flush()


def generate_statistics_and_report(records, events, output_dir):
    """根据录制数据生成结构化统计分析结果、Markdown 报告及可视化图表"""
    if not records:
        print("\n[提示] 没有采集到有效定位数据，跳过报告生成。")
        return

    os.makedirs(output_dir, exist_ok=True)
    csv_file = os.path.join(output_dir, "localization_raw_data.csv")
    events_file = os.path.join(output_dir, "localization_events.csv")
    report_file = os.path.join(output_dir, "localization_analysis_report.md")
    plot_file = os.path.join(output_dir, "localization_performance_analysis.png")

    # 1. 导出全量 CSV
    print(f"\n\n[1/4] 正在保存全量定位时序数据: {csv_file} ...")
    fieldnames = list(records[0].keys())
    with open(csv_file, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)
    print(f"      数据保存完成，共录制 {len(records)} 帧。")

    # 2. 导出事件 CSV
    if events:
        print(f"[2/4] 正在保存定位异常与切换事件明细: {events_file} ...")
        event_fields = ['time', 'type', 'detail', 'mode', 'speed']
        with open(events_file, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=event_fields)
            writer.writeheader()
            writer.writerows(events)
        print(f"      事件保存完成，共记录 {len(events)} 项异常/切换事件。")
    else:
        print("[2/4] 测试全程定位平稳，未检测到显著异常跳变事件。")

    # 3. 统计计算
    total_time = float(records[-1]['time'])
    sample_num = len(records)
    dt_avg = total_time / max(sample_num, 1)

    # 模式时长统计
    mode_counts = {}
    rtk_state_counts = {}
    speeds = []
    ndt_scores = []
    ndt_exe_times = []
    rtk_ndt_pos_diffs = []
    rtk_ndt_yaw_diffs = []
    ekf_ndt_pos_diffs = []
    jump_pos_vals = []
    jump_yaw_vals = []

    for r in records:
        m = r['mode']
        mode_counts[m] = mode_counts.get(m, 0) + 1

        rtk_st = r['rtk_state']
        rtk_state_counts[rtk_st] = rtk_state_counts.get(rtk_st, 0) + 1

        if r['speed_mps'] != '':
            speeds.append(float(r['speed_mps']))
        if r['ndt_score'] != '':
            ndt_scores.append(float(r['ndt_score']))
        if r['ndt_exe_time_ms'] != '':
            ndt_exe_times.append(float(r['ndt_exe_time_ms']))

        # 仅在 RTK 处于 FIXED 或高质量时统计 RTK vs NDT 物理差距
        if r['diff_rtk_ndt_pos_m'] != '' and r['rtk_state'] == 'FIXED':
            rtk_ndt_pos_diffs.append(float(r['diff_rtk_ndt_pos_m']))
        if r['diff_rtk_ndt_yaw_deg'] != '' and r['rtk_state'] == 'FIXED':
            rtk_ndt_yaw_diffs.append(float(r['diff_rtk_ndt_yaw_deg']))

        if r['diff_ekf_ndt_pos_m'] != '':
            ekf_ndt_pos_diffs.append(float(r['diff_ekf_ndt_pos_m']))

        if float(r['jump_pos_m']) > 0:
            jump_pos_vals.append(float(r['jump_pos_m']))
        if float(r['jump_yaw_deg']) > 0:
            jump_yaw_vals.append(float(r['jump_yaw_deg']))

    def safe_stats(arr):
        if not arr:
            return {'mean': 0.0, 'max': 0.0, 'min': 0.0, 'p95': 0.0}
        s_arr = sorted(arr)
        p95_idx = min(int(len(s_arr) * 0.95), len(s_arr) - 1)
        return {
            'mean': sum(s_arr) / len(s_arr),
            'max': max(s_arr),
            'min': min(s_arr),
            'p95': s_arr[p95_idx]
        }

    speed_st = safe_stats(speeds)
    score_st = safe_stats(ndt_scores)
    ndt_time_st = safe_stats(ndt_exe_times)
    pos_diff_st = safe_stats(rtk_ndt_pos_diffs)
    yaw_diff_st = safe_stats(rtk_ndt_yaw_diffs)

    # 4. 生成可视化图表 (Matplotlib)
    print(f"[3/4] 正在生成定位多源综合性能评估图表: {plot_file} ...")
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np

        times = np.array([float(r['time']) for r in records])

        def parse_series(key):
            res = []
            for r in records:
                v = r[key]
                res.append(float(v) if v != '' else np.nan)
            return np.array(res)

        fig, axes = plt.subplots(5, 1, figsize=(15, 14), sharex=True)
        fig.suptitle(f'Autoware Multi-Source Localization Performance & Stability Analysis\nTest Total: {total_time:.1f}s | Samples: {sample_num}', fontsize=14, fontweight='bold')

        # 子图 1: 模式与 RTK 状态
        ax1 = axes[0]
        mode_names = list(mode_counts.keys())
        mode_to_int = {m: i for i, m in enumerate(mode_names)}
        mode_series = [mode_to_int.get(r['mode'], 0) for r in records]
        ax1.step(times, mode_series, where='post', color='#4B0082', linewidth=2.0, label='Selected Pose Mode')
        ax1.set_yticks(range(len(mode_names)))
        ax1.set_yticklabels(mode_names)
        ax1.set_ylabel('Active Source', fontsize=10, fontweight='bold')
        ax1.grid(True, linestyle='--', alpha=0.5)

        # 标记 RTK 非固定解区域
        rtk_codes = np.array([int(r['rtk_code']) if r['rtk_code'] != '' else -1 for r in records])
        non_fixed = rtk_codes != 4
        if np.any(non_fixed):
            ax1_twin = ax1.twinx()
            ax1_twin.plot(times, rtk_codes, color='#FF4500', linestyle=':', label='RTK Code (4=Fixed, 2=DGPS)')
            ax1_twin.set_ylabel('RTK State', color='#FF4500')

        # 子图 2: 车速与跳变标记
        ax2 = axes[1]
        ax2.plot(times, parse_series('speed_mps'), color='#2E8B57', linewidth=1.5, label='Speed (m/s)')
        # 标出跳跃点
        jump_flags = np.array([int(r['is_jump_event']) for r in records])
        if np.any(jump_flags > 0):
            jump_times = times[jump_flags > 0]
            jump_speeds = parse_series('speed_mps')[jump_flags > 0]
            ax2.scatter(jump_times, jump_speeds, color='red', marker='x', s=60, zorder=5, label='Pose Jump Anomaly')
        ax2.set_ylabel('Speed (m/s)', fontsize=10, fontweight='bold')
        ax2.grid(True, linestyle='--', alpha=0.5)
        ax2.legend(loc='upper right')

        # 子图 3: 航向角比对 (RTK vs NDT vs EKF)
        ax3 = axes[2]
        ax3.plot(times, parse_series('rtk_yaw_deg'), color='#DC143C', linestyle='-.', label='RTK Heading')
        ax3.plot(times, parse_series('ndt_yaw_deg'), color='#4169E1', linestyle='--', label='NDT Heading')
        ax3.plot(times, parse_series('ekf_yaw_deg'), color='#006400', linewidth=2.0, label='EKF Output Heading')
        ax3.set_ylabel('Yaw (deg)', fontsize=10, fontweight='bold')
        ax3.set_ylim(-5, 365)
        ax3.grid(True, linestyle='--', alpha=0.5)
        ax3.legend(loc='upper right')

        # 子图 4: 定位源偏差 (RTK vs NDT 间距与偏角)
        ax4 = axes[3]
        ax4.plot(times, parse_series('diff_rtk_ndt_pos_m'), color='#FF8C00', label='RTK-NDT Pos Diff (m)')
        ax4.axhline(0.3, color='red', linestyle='--', alpha=0.7, label='0.3m Warning Bound')
        ax4.set_ylabel('Pos Diff (m)', fontsize=10, fontweight='bold')
        ax4.grid(True, linestyle='--', alpha=0.5)
        ax4.legend(loc='upper left')

        ax4_t = ax4.twinx()
        ax4_t.plot(times, parse_series('diff_rtk_ndt_yaw_deg'), color='#800080', linestyle=':', label='RTK-NDT Yaw Diff (deg)')
        ax4_t.set_ylabel('Yaw Diff (deg)', color='#800080')

        # 子图 5: NDT 得分与耗时
        ax5 = axes[4]
        ax5.plot(times, parse_series('ndt_score'), color='#1E90FF', label='NDT Transform Probability (Score)')
        ax5.axhline(2.0, color='crimson', linestyle='--', alpha=0.7, label='Score=2.0 Safe Bound')
        ax5.set_ylabel('NDT Score', fontsize=10, fontweight='bold')
        ax5.set_xlabel('Elapsed Time (seconds)', fontsize=11, fontweight='bold')
        ax5.grid(True, linestyle='--', alpha=0.5)
        ax5.legend(loc='upper left')

        plt.tight_layout()
        plt.savefig(plot_file, dpi=200)
        print(f"      图表生成成功: {plot_file}")
    except Exception as e:
        print(f"      [提示] 图表生成跳过或出现异常: {e}")

    # 5. 生成专业 Markdown 报告
    print(f"[4/4] 正在生成综合诊断分析报告: {report_file} ...")
    md_content = f"""# Autoware 多定位源数据记录与稳定性综合分析报告

- **测试生成时间**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
- **测试总时长**: {total_time:.2f} 秒
- **总采样帧数**: {sample_num} 帧 (平均采样率: {1.0/dt_avg:.1f} Hz)
- **原始数据保存路径**: `{csv_file}`
- **可视化图表路径**: `{plot_file}`

---

## 1. 车辆运动态概况
| 指标项 | 统计值 | 说明 |
| :--- | :--- | :--- |
| **平均行驶车速** | `{speed_st['mean']:.2f} m/s` ({speed_st['mean']*3.6:.1f} km/h) | 测试期间有效平均车速 |
| **最高车速** | `{speed_st['max']:.2f} m/s` ({speed_st['max']*3.6:.1f} km/h) | 峰值车速 |
| **静止/低速时间占比** | `{sum(1 for s in speeds if s < 0.1)/max(len(speeds),1)*100:.1f} %` | 车速小于 0.1 m/s 的帧数比例 |

---

## 2. 定位模式与状态机运行分布
各定位模式被激活使用的时间分布如下：
"""
    for m, count in sorted(mode_counts.items(), key=lambda x: -x[1]):
        dur = count * dt_avg
        pct = (count / sample_num) * 100
        md_content += f"- **`{m}`**: `{dur:.1f}s` (`{pct:.1f}%`)\n"

    md_content += "\n### RTK 定位状态分布 (NavSatFix 状态)\n"
    for st, count in sorted(rtk_state_counts.items(), key=lambda x: -x[1]):
        dur = count * dt_avg
        pct = (count / sample_num) * 100
        md_content += f"- **`{st}`**: `{dur:.1f}s` (`{pct:.1f}%`)\n"

    md_content += f"""
- **RTK 失锁/降级发生次数**: `{self.rtk_loss_count if 'self' in locals() else len([e for e in events if 'RTK失锁' in e['type']])} 次`
- **定位模式切换次数**: `{self.mode_change_count if 'self' in locals() else len([e for e in events if '模式切换' in e['type']])} 次`

---

## 3. 多定位源空间基准差距分析 (RTK vs NDT)
> **统计口径**：仅在 RTK 处于高质量固定解 (`FIXED`) 期间计算，避免将 RTK 自身丢星漂移计入真实物理偏差。

| 评估指标 | 平均值 | 95分位值 (P95) | 最大偏差 | 评价建议门限 | 评估结论 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **水平空间距离差** | `{pos_diff_st['mean']:.3f} m` | `{pos_diff_st['p95']:.3f} m` | `{pos_diff_st['max']:.3f} m` | `<= 0.20 m` | {'✅ 良好' if pos_diff_st['mean'] <= 0.2 else '⚠️ 偏大(需校准)'} |
| **航向角偏差 (Yaw)** | `{yaw_diff_st['mean']:.2f} °` | `{yaw_diff_st['p95']:.2f} °` | `{yaw_diff_st['max']:.2f} °` | `<= 2.50 °` | {'✅ 良好' if yaw_diff_st['mean'] <= 2.5 else '⚠️ 偏大(需校准天线/外参)'} |

---

## 4. NDT 匹配质量与得分统计
| 指标项 | 统计值 | 评价基准 | 状态诊断 |
| :--- | :--- | :--- | :--- |
| **NDT 变换概率均值 (Score)** | `{score_st['mean']:.3f}` | 良好通常 `>= 2.5` | {'✅ 优秀' if score_st['mean'] >= 2.5 else '⚠️ 偏低'} |
| **NDT 变换概率最低值 (Min)** | `{score_st['min']:.3f}` | 低于 `1.5` 易发生失锁 | {'✅ 正常' if score_st['min'] >= 1.5 else '🚨 存在危险低分配准'} |
| **NDT 平均配准耗时** | `{ndt_time_st['mean']:.2f} ms` | 应 `<= 35 ms` | {'✅ 实时性好' if ndt_time_st['mean'] <= 35 else '⚠️ 计算耗时偏长'} |
| **NDT 最大单帧耗时** | `{ndt_time_st['max']:.2f} ms` | 避免超过 `100 ms` | {'✅ 正常' if ndt_time_st['max'] <= 100 else '🚨 存在严重超时卡顿'} |

---

## 5. 定位稳定性与跳变事件记录
- **总跳变与突变告警次数**: `{len(events)} 项`
"""
    if events:
        md_content += "\n### 典型突变与异常事件列表（按时序排列）\n"
        md_content += "| 相对时间 | 事件类型 | 详细描述 | 当时模式 | 当时车速 |\n"
        md_content += "| :--- | :--- | :--- | :--- | :--- |\n"
        for e in events[:30]:  # 最多输出前30条典型事件
            md_content += f"| `{e['time']}s` | **{e['type']}** | {e['detail']} | `{e['mode']}` | `{e['speed']} m/s` |\n"
        if len(events) > 30:
            md_content += f"| ... | 更多事件已截断 | 详情请查看 `{events_file}` | ... | ... |\n"
    else:
        md_content += "\n> 本次测试未触发任何位置或航向剧烈突变，定位连续平稳。\n"

    md_content += """
---

## 6. 核心定位问题诊断与优化建议
1. **关于多源空间基准统一（RTK 与 NDT 偏差）**：
   - 若实测 `RTK-NDT 航向差 > 3°` 或 `位置差 > 0.25m`，说明激光点云地图的坐标原点/投影参数，或双天线 RTK 安装外参（`base_link` 至 GNSS 天线）存在系统性偏置。建议在开阔直道进行外参微调对齐，消除系统固定残差。
2. **关于进室内/遮挡区失锁保护（平滑退化机制）**：
   - 检查 `pose_covariance_modifier` 在 RTK 降级（state!=4 或 stddev>0.25）切回 NDT 时，应避免瞬时硬切。若检测到在切换时刻伴随有 `EKF位置突变`，需增加位姿平滑交接窗口，防止下游控制器出现阶跃打舵。
3. **关于 NDT 激光点云匹配性能**：
   - 若在特定路段 NDT 得分骤降（<1.5），说明该路段几何特征单一（长廊长直道）或点云地图在该区域存在点云空洞/遮挡。建议补充该路段的稠密点云地图，或调整体素滤波网格大小。
"""

    with open(report_file, 'w', encoding='utf-8') as f:
        f.write(md_content)

    print(f"\n====================== 定位评估诊断摘要 ======================")
    print(f"• 测试时长: {total_time:.1f}s | 采样: {sample_num}帧 | 模式数: {len(mode_counts)}")
    print(f"• RTK 固定解(Fixed)占比: {(rtk_state_counts.get('FIXED', 0)/sample_num)*100:4.1f}%")
    print(f"• NDT 平均得分: {score_st['mean']:.3f} (最低: {score_st['min']:.3f})")
    print(f"• RTK与NDT平均位置差: {pos_diff_st['mean']:.3f}m | 最大: {pos_diff_st['max']:.3f}m")
    print(f"• RTK与NDT平均航向差: {yaw_diff_st['mean']:.2f}° | 最大: {yaw_diff_st['max']:.2f}°")
    print(f"• 异常跳变与告警事件: 共 {len(events)} 项")
    print(f"• 完整诊断报告已生成: {report_file}")
    print(f"==============================================================\n")


def offline_analysis_from_csv(csv_path):
    """从已存在的 CSV 文件中解析并生成报告与图表"""
    if not os.path.exists(csv_path):
        print(f"[错误] 指定的 CSV 文件不存在: {csv_path}")
        return

    print(f"[离线模式] 正在读取并分析历史 CSV 数据: {csv_path} ...")
    records = []
    events = []
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for r in reader:
            records.append(r)
            if r.get('is_jump_event') == '1':
                events.append({
                    'time': r['time'],
                    'type': '定位跳变历史记录',
                    'detail': f"位置跳跃: {r.get('jump_pos_m', '0')}m, 航向跳跃: {r.get('jump_yaw_deg', '0')}°",
                    'mode': r['mode'],
                    'speed': r['speed_mps']
                })

    out_dir = os.path.dirname(os.path.abspath(csv_path))
    generate_statistics_and_report(records, events, out_dir)


def main():
    parser = argparse.ArgumentParser(description="Autoware 多定位源数据记录与稳定性分析工具")
    parser.add_argument("csv_file", nargs="?", default=None, help="[可选] 离线分析模式：指定已有 CSV 文件路径")
    parser.add_argument("--hz", type=float, default=20.0, help="采样频率 (默认 20.0 Hz)")
    parser.add_argument("--out-dir", type=str, default=None, help="输出目录 (默认保存在 ~/autoware/log/localization_eval/YYYYMMDD_HHMMSS/)")
    cli_args = parser.parse_args()

    # 1. 离线 CSV 分析模式
    if cli_args.csv_file:
        offline_analysis_from_csv(cli_args.csv_file)
        return

    # 2. 实时在线采集模式
    if rclpy is None:
        print("[错误] 未检测到 ROS 2 Python 环境 (rclpy)！")
        print("• 实时采集模式下，请先执行: source /opt/ros/humble/setup.bash && source install/setup.bash")
        print("• 离线分析模式下，请直接传入 CSV 路径: python3 record_and_analyze_localization.py <path_to_csv>")
        sys.exit(1)

    now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_directory = cli_args.out_dir
    if output_directory is None:
        output_directory = f"/home/nvidia/autoware/log/localization_eval/{now_str}"

    os.makedirs(output_directory, exist_ok=True)

    print("==================================================================")
    print("      Autoware 多定位源数据记录、稳定性监控与综合诊断分析工具      ")
    print("==================================================================")
    print(f"• 采样频率: {cli_args.hz} Hz")
    print(f"• 保存目录: {output_directory}")
    print("• 监控源  : RTK (/sensing/gnss/*), NDT (/localization/pose_estimator/*), EKF (/localization/kinematic_state)")
    print("• 操作提示: 保持运行进行实车或仿真测试，测试结束时按 【Ctrl + C】 即可自动生成分析报告与图表。")
    print("==================================================================\n")

    rclpy.init()
    node = LocalizationRecorderNode(sample_rate_hz=cli_args.hz, log_root=output_directory)

    def sig_handler(sig, frame):
        node.destroy_node()
        rclpy.shutdown()
        generate_statistics_and_report(node.records, node.events, output_directory)
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        generate_statistics_and_report(node.records, node.events, output_directory)


if __name__ == '__main__':
    main()
