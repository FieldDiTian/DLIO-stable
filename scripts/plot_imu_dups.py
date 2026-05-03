#!/usr/bin/env python3
"""Plot /vks/imu duplicate-timestamp pattern from a rosbag2 mcap.

Usage:
  plot_imu_dups.py <bag_path> [topic] [--max <N>] [--out <png>]
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


def read_imu(bag_path: str, topic: str, max_msgs: int):
    opts = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    conv = rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                       output_serialization_format="cdr")
    r = rosbag2_py.SequentialReader()
    r.open(opts, conv)
    f = rosbag2_py.StorageFilter()
    f.topics = [topic]
    r.set_filter(f)

    recv_ns = []
    hdr_ns = []
    while r.has_next() and (max_msgs <= 0 or len(recv_ns) < max_msgs):
        tp, data, t = r.read_next()
        if tp != topic:
            continue
        msg = rclpy.serialization.deserialize_message(data, Imu)
        recv_ns.append(int(t))
        hdr_ns.append(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
    return np.array(recv_ns, dtype=np.int64), np.array(hdr_ns, dtype=np.int64)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("bag")
    p.add_argument("topic", nargs="?", default="/vks/imu")
    p.add_argument("--max", type=int, default=20000,
                   help="max messages to read (0 = all)")
    p.add_argument("--out", default="/tmp/imu_dups.png")
    args = p.parse_args()

    print(f"reading {args.topic} from {args.bag} (max={args.max})...")
    recv, hdr = read_imu(args.bag, args.topic, args.max)
    if len(recv) == 0:
        print("no messages")
        return 1
    n = len(recv)
    t0 = recv[0]
    recv_s = (recv - t0) / 1e9
    hdr_s = (hdr - t0) / 1e9

    # Per-message gap between consecutive header stamps (delta_hdr) and
    # consecutive recv stamps (delta_recv).
    d_hdr_ms = np.diff(hdr) / 1e6
    d_recv_ms = np.diff(recv) / 1e6

    # Duplicate header.stamp = streak length analysis.
    streak_lens = []
    cur = 1
    for i in range(1, n):
        if hdr[i] == hdr[i - 1]:
            cur += 1
        else:
            streak_lens.append(cur)
            cur = 1
    streak_lens.append(cur)
    streak_lens = np.array(streak_lens)
    dup_count = (np.diff(hdr) == 0).sum()
    pub_rate = (n - 1) / ((recv[-1] - recv[0]) / 1e9)
    uniq_rate = (n - 1 - dup_count) / ((recv[-1] - recv[0]) / 1e9)

    print(f"n_messages          = {n}")
    print(f"duplicate_count     = {dup_count} ({100.0*dup_count/(n-1):.1f}%)")
    print(f"published_rate_Hz   = {pub_rate:.2f}")
    print(f"unique_stamp_rate_Hz= {uniq_rate:.2f}")
    print(f"streak_len mean={streak_lens.mean():.2f} max={streak_lens.max()}")

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=False)
    fig.suptitle(f"{args.topic} timestamp diagnostics  ({n} msgs, "
                 f"pub={pub_rate:.1f}Hz, unique={uniq_rate:.1f}Hz, "
                 f"dup={100.0*dup_count/(n-1):.1f}%)")

    # 1. Header-stamp delta over time (zero = duplicate stamp).
    ax = axes[0]
    ax.plot(recv_s[1:], d_hdr_ms, ",", color="#1f77b4", alpha=0.6)
    ax.axhline(0, color="red", lw=0.5, alpha=0.5)
    ax.set_ylabel("header.stamp delta (ms)")
    ax.set_xlabel("recv-time (s, relative)")
    ax.set_ylim(-2, max(50, np.percentile(d_hdr_ms, 99) * 1.2))
    ax.set_title("Per-message header.stamp delta — points at 0 are duplicates")
    ax.grid(alpha=0.3)

    # 2. Histogram of header-stamp deltas.
    ax = axes[1]
    bins = np.arange(0, 60, 1)
    ax.hist(d_hdr_ms, bins=bins, color="#1f77b4", alpha=0.7, edgecolor="white")
    ax.set_xlabel("header.stamp delta (ms)")
    ax.set_ylabel("count")
    ax.set_title("Histogram of header.stamp deltas")
    ax.grid(alpha=0.3)

    # 3. Streak-length histogram.
    ax = axes[2]
    max_streak = streak_lens.max()
    bins3 = np.arange(0.5, max_streak + 1.5, 1)
    ax.hist(streak_lens, bins=bins3, color="#d62728", alpha=0.7, edgecolor="white")
    ax.set_xlabel("duplicate streak length (consecutive msgs sharing a stamp)")
    ax.set_ylabel("count")
    ax.set_title(f"Streak lengths (mean={streak_lens.mean():.2f}, max={max_streak})")
    ax.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(args.out, dpi=120)
    print(f"saved {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
