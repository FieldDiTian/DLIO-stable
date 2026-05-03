#!/usr/bin/env python3
"""Visualize IMU header.stamp vs recv_time inside a specific time window.

Usage:
  plot_imu_window.py <bag> <topic> <t_lo> <t_hi> [--out <png>]

Example:
  plot_imu_window.py bag.mcap /vks/imu 1776279837.99 1776279838.05
"""
import sys
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rosbag2_py
import rclpy.serialization
from sensor_msgs.msg import Imu


def main():
    p = argparse.ArgumentParser()
    p.add_argument("bag")
    p.add_argument("topic")
    p.add_argument("t_lo", type=float)
    p.add_argument("t_hi", type=float)
    p.add_argument("--out", default="/tmp/imu_window.png")
    args = p.parse_args()

    opts = rosbag2_py.StorageOptions(uri=args.bag, storage_id="mcap")
    conv = rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                       output_serialization_format="cdr")
    r = rosbag2_py.SequentialReader()
    r.open(opts, conv)
    f = rosbag2_py.StorageFilter()
    f.topics = [args.topic]
    r.set_filter(f)

    recv = []
    hdr = []
    while r.has_next():
        tp, data, t = r.read_next()
        if tp != args.topic:
            continue
        msg = rclpy.serialization.deserialize_message(data, Imu)
        h = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if args.t_lo <= h <= args.t_hi:
            recv.append(t / 1e9)
            hdr.append(h)
        elif h > args.t_hi + 0.5:
            break

    if not recv:
        print("no messages in window")
        return 1

    recv = np.array(recv)
    hdr = np.array(hdr)

    # Compute duplicate runs.
    uniq, counts = np.unique(hdr, return_counts=True)

    print(f"window [{args.t_lo:.6f}, {args.t_hi:.6f}]  ({(args.t_hi-args.t_lo)*1000:.1f} ms)")
    print(f"messages: {len(recv)}   unique stamps: {len(uniq)}   duplicates: {len(recv) - len(uniq)}")
    print()
    print(f"{'recv_time':<20} {'header.stamp':<20} dup_index")
    last_h = None
    dup_i = 0
    for r_t, h in zip(recv, hdr):
        if h == last_h:
            dup_i += 1
            tag = f" <- dup #{dup_i}"
        else:
            dup_i = 0
            tag = ""
        last_h = h
        print(f"{r_t:<20.6f} {h:<20.6f}{tag}")

    fig, ax = plt.subplots(figsize=(11, 6))
    fig.suptitle(f"{args.topic}  window [{args.t_lo:.3f}, {args.t_hi:.3f}]   "
                 f"({len(recv)} msgs, {len(uniq)} unique stamps)")

    # x = recv_time, y = header.stamp.
    # Color groups by duplicate stamp.
    color_map = {}
    next_color = 0
    palette = plt.cm.tab10.colors
    colors = []
    for h in hdr:
        if h not in color_map:
            color_map[h] = palette[next_color % 10]
            next_color += 1
        colors.append(color_map[h])

    ax.scatter(recv, hdr, c=colors, s=80, edgecolors="black", linewidths=0.6, zorder=3)

    # Horizontal lines for each unique stamp + label with multiplicity.
    for h, c in zip(uniq, counts):
        col = color_map[h]
        ax.axhline(h, color=col, alpha=0.35, lw=1.2)
        ax.text(args.t_hi + (args.t_hi-args.t_lo)*0.01, h,
                f"  ×{c}",
                va="center", color=col, fontsize=10, fontweight="bold")

    # 45-degree reference: recv_time == header.stamp.
    lo = min(args.t_lo, recv.min())
    hi = max(args.t_hi, recv.max())
    ax.plot([lo, hi], [lo, hi], "k--", lw=0.6, alpha=0.4, label="recv_time = header.stamp")

    ax.set_xlabel("recv_time (s, absolute)")
    ax.set_ylabel("header.stamp (s, absolute)")
    ax.legend(loc="lower right")
    ax.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(args.out, dpi=140)
    print(f"\nsaved {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
