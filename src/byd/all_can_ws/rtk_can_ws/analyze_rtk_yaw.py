#!/usr/bin/env python3
"""
RTK Yaw Stability Analyzer
Usage: python3 analyze_rtk_yaw.py <path_to_rosbag>

This script analyzes ROS 2 bags for RTK yaw "staircase" issues (where values 
stay constant for ~1s and then jump). It generates diagnostic plots and 
statistics.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# ROS 2 imports
try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from autoware_sensing_msgs.msg import GnssInsOrientationStamped
except ImportError:
    print("Error: ROS 2 environment not sourced or dependencies missing.")
    print("Please run: source /opt/ros/<distro>/setup.bash")
    sys.exit(1)

def get_rosbag_options(path, storage_id='sqlite3', serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)
    return storage_options, converter_options

def run_analysis(bag_path):
    print(f"Reading bag: {bag_path}")
    if not os.path.exists(bag_path):
        print(f"File not found: {bag_path}")
        return

    storage_options, converter_options = get_rosbag_options(bag_path)
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    timestamps = []
    yaws = []

    # 1. Data Extraction
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == '/autoware_orientation':
            msg = deserialize_message(data, GnssInsOrientationStamped)
            
            # Timestamp from header
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            timestamps.append(stamp)
            
            # Quaternion to Euler (Yaw)
            q = msg.orientation.orientation
            r = R.from_quat([q.x, q.y, q.z, q.w])
            yaw = r.as_euler('zyx')[0] 
            yaws.append(np.degrees(yaw))

    if not timestamps:
        print("No messages found on /autoware_orientation")
        return

    timestamps = np.array(timestamps)
    yaws = np.array(yaws)
    
    # Sort data by time
    idx = np.argsort(timestamps)
    timestamps = timestamps[idx]
    yaws = yaws[idx]
    
    t_rel = timestamps - timestamps[0]
    duration = t_rel[-1]
    
    # 2. Statistics & Plateau Detection
    print(f"\n--- Analysis Results ---")
    print(f"Total Messages: {len(yaws)}")
    print(f"Duration:       {duration:.2f} s")
    print(f"Avg Frequency:  {len(yaws)/duration:.2f} Hz")

    epsilon = 1e-9
    plateau_counts = 0
    plateau_durations = []
    curr_start = 0
    
    for i in range(1, len(yaws)):
        if abs(yaws[i] - yaws[i-1]) > epsilon:
            dur = timestamps[i-1] - timestamps[curr_start]
            if dur > 0.8: # Threshold for the reported 1s issue
                plateau_counts += 1
                plateau_durations.append(dur)
            curr_start = i

    print(f"Detected {plateau_counts} plateaus > 0.8s (indicative of the issue)")
    if plateau_durations:
        print(f"Avg plateau duration: {np.mean(plateau_durations):.3f} s")
        print(f"Max plateau duration: {np.max(plateau_durations):.3f} s")

    # 3. Plotting
    prefix = os.path.basename(bag_path.rstrip('/'))
    
    def create_line_plot(t, y, title, filename):
        plt.figure(figsize=(15, 6))
        plt.plot(t, y, 'b-', linewidth=1.2)
        plt.title(title)
        plt.xlabel('Time (s)')
        plt.ylabel('Yaw (deg)')
        plt.grid(True, alpha=0.3)
        plt.savefig(filename)
        plt.close()
        print(f"Saved: {filename}")

    # Full Plot
    create_line_plot(t_rel, yaws, f"Full Yaw Plot - {prefix}", f"{prefix}_full.png")

    # Middle 2 Min
    mid = duration / 2
    mask_mid = (t_rel >= max(0, mid-60)) & (t_rel <= min(duration, mid+60))
    if np.any(mask_mid):
        create_line_plot(t_rel[mask_mid], yaws[mask_mid], 
                         f"Middle 2 Minutes - {prefix}", f"{prefix}_mid_2min.png")

    # Last 2 Min
    mask_last = (t_rel >= max(0, duration-120))
    if np.any(mask_last):
        create_line_plot(t_rel[mask_last], yaws[mask_last], 
                         f"Last 2 Minutes - {prefix}", f"{prefix}_last_2min.png")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_rtk_yaw.py <bag_path>")
    else:
        run_analysis(sys.argv[1])
