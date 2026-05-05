#!/usr/bin/env python3
"""
Plot the localized path and mark source-switch points.

The script reads /gicp/localization/odom and /gicp/localization/source_status,
plots the XY path, and annotates each time the active source changes between
GICP and GLOBAL ODOM.
"""

import argparse
import math
import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(os.environ.get("TMPDIR", "/tmp")) / "gicp_source_switch_matplotlib"),
)

try:
    import matplotlib
    if "--live" not in sys.argv:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as exc:
    print(
        "matplotlib is required for plotting. Install it with: sudo apt install python3-matplotlib",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc

try:
    import rosbag2_py
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from visualization_msgs.msg import MarkerArray
except ImportError as exc:
    print(
        "ROS 2 Python bag/message modules are required. Source your ROS 2 environment first.",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


PathPoint = Tuple[float, float, float]
SourcePoint = Tuple[float, str, Optional[float], Optional[float]]

COLORS = {
    "GICP": "limegreen",
    "GLOBAL_ODOM": "deeppink",
    "IMU_PRIOR": "orange",
}


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def bag_time_to_sec(timestamp_ns: int) -> float:
    return float(timestamp_ns) * 1e-9


def guess_storage_id(bag_path: Path) -> str:
    if bag_path.is_file() and bag_path.suffix == ".mcap":
        return "mcap"
    if bag_path.is_file() and bag_path.suffix in (".db3", ".sqlite3"):
        return "sqlite3"

    metadata = bag_path / "metadata.yaml"
    if metadata.exists():
        for line in metadata.read_text(encoding="utf-8", errors="replace").splitlines():
            stripped = line.strip()
            if stripped.startswith("storage_identifier:"):
                value = stripped.split(":", 1)[1].strip().strip("'\"")
                if value:
                    return value

    if bag_path.is_dir() and any(bag_path.glob("*.mcap")):
        return "mcap"
    return "sqlite3"


def open_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id=guess_storage_id(bag_path),
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def parse_source_sample(marker_array: MarkerArray) -> Tuple[Optional[str], Optional[float], Optional[float]]:
    fallback_xy: Tuple[Optional[float], Optional[float]] = (None, None)
    for marker in marker_array.markers:
        if marker.action != marker.ADD:
            continue
        if fallback_xy == (None, None):
            fallback_xy = (marker.pose.position.x, marker.pose.position.y)
        text = marker.text.strip().upper()
        if not text:
            continue
        first_line = text.splitlines()[0]
        if "GLOBAL ODOM" in first_line:
            return "GLOBAL_ODOM", marker.pose.position.x, marker.pose.position.y
        if "GICP" in first_line:
            return "GICP", marker.pose.position.x, marker.pose.position.y
        if "IMU" in first_line:
            return "IMU_PRIOR", marker.pose.position.x, marker.pose.position.y
    return None, fallback_xy[0], fallback_xy[1]


def parse_source(marker_array: MarkerArray) -> Optional[str]:
    source, _x, _y = parse_source_sample(marker_array)
    return source


def read_bag(
    bag_path: Path,
    odom_topic: str,
    pose_topic: str,
    source_topic: str,
) -> Tuple[List[PathPoint], List[SourcePoint]]:
    reader = open_reader(bag_path)
    topics: Dict[str, str] = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}

    requested_path_topics = [topic for topic in (odom_topic, pose_topic) if topic]
    path_topics = [topic for topic in requested_path_topics if topic in topics]
    missing = [source_topic] if source_topic not in topics else []
    if not path_topics:
        missing.append(" or ".join(requested_path_topics))
    if missing:
        available = "\n  ".join(sorted(topics))
        raise RuntimeError(f"Missing topic(s): {', '.join(missing)}\nAvailable topics:\n  {available}")

    storage_filter = rosbag2_py.StorageFilter(topics=[*path_topics, source_topic])
    reader.set_filter(storage_filter)

    path_points: List[PathPoint] = []
    source_points: List[SourcePoint] = []

    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()
        if topic == odom_topic:
            msg = deserialize_message(data, Odometry)
            msg_time = stamp_to_sec(msg.header.stamp)
            t = msg_time if msg_time > 0.0 else bag_time_to_sec(bag_time_ns)
            path_points.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y))
        elif topic == pose_topic:
            msg = deserialize_message(data, PoseStamped)
            msg_time = stamp_to_sec(msg.header.stamp)
            t = msg_time if msg_time > 0.0 else bag_time_to_sec(bag_time_ns)
            path_points.append((t, msg.pose.position.x, msg.pose.position.y))
        elif topic == source_topic:
            msg = deserialize_message(data, MarkerArray)
            source, x, y = parse_source_sample(msg)
            if source:
                # Marker stamps are generated from the odom/localization stamp.
                stamp = msg.markers[0].header.stamp if msg.markers else None
                marker_time = stamp_to_sec(stamp) if stamp is not None else 0.0
                t = marker_time if marker_time > 0.0 else bag_time_to_sec(bag_time_ns)
                source_points.append((t, source, x, y))

    return path_points, source_points


def nearest_path(path_points: Sequence[PathPoint], t: float) -> Optional[PathPoint]:
    if not path_points:
        return None
    return min(path_points, key=lambda point: abs(point[0] - t))


def source_switches(source_points: Sequence[SourcePoint]) -> List[SourcePoint]:
    switches: List[SourcePoint] = []
    last_source: Optional[str] = None
    for point in source_points:
        _t, source, _x, _y = point
        if source != last_source:
            if last_source is not None:
                switches.append(point)
            last_source = source
    return switches


def plot(
    path_points: Sequence[PathPoint],
    source_points: Sequence[SourcePoint],
    output_path: Path,
    title: str,
) -> None:
    if not path_points:
        raise RuntimeError("No path samples found.")

    switches = source_switches(source_points)
    xs = [point[1] for point in path_points]
    ys = [point[2] for point in path_points]

    fig, ax = plt.subplots(figsize=(11, 9))
    ax.plot(xs, ys, color="black", linewidth=1.2, label="localization path")
    ax.scatter(xs[0], ys[0], color="dodgerblue", s=55, marker="o", label="start", zorder=3)
    ax.scatter(xs[-1], ys[-1], color="black", s=55, marker="x", label="end", zorder=3)

    for index, (t, source, source_x, source_y) in enumerate(switches, start=1):
        if source_x is not None and source_y is not None:
            x, y = source_x, source_y
        else:
            point = nearest_path(path_points, t)
            if point is None:
                continue
            _, x, y = point
        if x is None or y is None:
            continue
        color = COLORS.get(source, "gold")
        ax.scatter(x, y, color=color, edgecolor="white", linewidth=0.8, s=90, zorder=4)
        ax.annotate(
            f"{index}: {source}",
            xy=(x, y),
            xytext=(6, 6),
            textcoords="offset points",
            fontsize=8,
            color=color,
            weight="bold",
        )

    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=160)
    plt.close(fig)

    print(f"Wrote {output_path}")
    print(f"Path samples: {len(path_points)}")
    print(f"Source samples: {len(source_points)}")
    print(f"Switches: {len(switches)}")
    for index, (t, source, source_x, source_y) in enumerate(switches, start=1):
        if source_x is not None and source_y is not None:
            x, y = source_x, source_y
        else:
            point = nearest_path(path_points, t)
            if point is None:
                continue
            _, x, y = point
        if x is None or y is None:
            continue
        print(f"  {index:02d} t={t:.3f} source={source} xy=({x:.3f}, {y:.3f})")


class LiveSwitchPlotter(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("gicp_source_switch_plotter")
        self.args = args
        self.path_points: List[PathPoint] = []
        self.source_points: List[SourcePoint] = []
        self.switches: List[SourcePoint] = []
        self.last_source: Optional[str] = None
        self.path_seen = False
        self.source_seen = False
        self.start_wall_time = time.monotonic()
        self.last_wait_log_time = 0.0

        self.fig, self.ax = plt.subplots(figsize=(11, 9))
        self.path_line, = self.ax.plot([], [], color="black", linewidth=1.2, label="localization path")
        self.start_line, = self.ax.plot(
            [], [], color="dodgerblue", marker="o", linestyle="None", markersize=7, label="start", zorder=5
        )
        self.latest_line, = self.ax.plot(
            [], [], color="black", marker="x", linestyle="None", markersize=7, label="latest", zorder=5
        )
        self.switch_lines = {}
        for source, color in COLORS.items():
            self.switch_lines[source], = self.ax.plot(
                [],
                [],
                color=color,
                marker="o",
                linestyle="None",
                markersize=8,
                markeredgecolor="white",
                markeredgewidth=0.9,
                label=f"switch to {source}",
                zorder=6,
            )
        self.switch_artists = []
        self.waiting_text = self.ax.text(
            0.5,
            0.5,
            f"Waiting for path samples on {args.odom_topic}",
            transform=self.ax.transAxes,
            ha="center",
            va="center",
            fontsize=14,
            color="0.35",
        )

        self.ax.set_title(args.title)
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.grid(True, alpha=0.25)
        self.ax.legend(loc="best")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        if args.odom_topic:
            self.create_subscription(Odometry, args.odom_topic, self.odom_callback, qos)
        if args.pose_topic:
            self.create_subscription(PoseStamped, args.pose_topic, self.pose_callback, qos)
        self.create_subscription(MarkerArray, args.source_topic, self.source_callback, qos)
        self.create_timer(args.update_period, self.update_plot)

        plt.ion()
        self.fig.show()
        self.get_logger().info(f"Tracing path from odom topic {args.odom_topic}")
        if args.pose_topic:
            self.get_logger().info(f"Also accepting optional pose path samples from {args.pose_topic}")
        self.get_logger().info(f"Listening for source status on {args.source_topic}")
        self.log_topic_diagnostics()

    def odom_callback(self, msg: Odometry) -> None:
        msg_time = stamp_to_sec(msg.header.stamp)
        t = msg_time if msg_time > 0.0 else self.get_clock().now().nanoseconds * 1e-9
        self.add_path_point(t, msg.pose.pose.position.x, msg.pose.pose.position.y, self.args.odom_topic, msg.header.frame_id)

    def pose_callback(self, msg: PoseStamped) -> None:
        msg_time = stamp_to_sec(msg.header.stamp)
        t = msg_time if msg_time > 0.0 else self.get_clock().now().nanoseconds * 1e-9
        self.add_path_point(t, msg.pose.position.x, msg.pose.position.y, self.args.pose_topic, msg.header.frame_id)

    def add_path_point(self, t: float, x: float, y: float, topic: str, frame_id: str) -> None:
        self.path_points.append((t, x, y))
        if not self.path_seen:
            self.path_seen = True
            if self.waiting_text is not None:
                self.waiting_text.remove()
                self.waiting_text = None
            self.get_logger().info(
                f"received first path sample from {topic} frame={frame_id} xy=({x:.3f}, {y:.3f})"
            )

    def source_callback(self, msg: MarkerArray) -> None:
        source, x, y = parse_source_sample(msg)
        if source is None:
            return
        if not self.source_seen:
            self.source_seen = True
            self.get_logger().info(f"received first source-status sample: {source}")
        stamp = msg.markers[0].header.stamp if msg.markers else None
        marker_time = stamp_to_sec(stamp) if stamp is not None else 0.0
        t = marker_time if marker_time > 0.0 else self.get_clock().now().nanoseconds * 1e-9
        self.source_points.append((t, source, x, y))
        if self.last_source is None:
            self.last_source = source
            return
        if source != self.last_source:
            self.switches.append((t, source, x, y))
            self.last_source = source
            if x is None or y is None:
                point = nearest_path(self.path_points, t)
                if point is not None:
                    _, x, y = point
            if x is not None and y is not None:
                self.get_logger().info(
                    f"source switch -> {source} at t={t:.3f} xy=({x:.3f}, {y:.3f})"
                )

    def update_plot(self) -> None:
        if not self.path_points:
            self.maybe_log_waiting()
            plt.pause(0.001)
            return

        xs = [point[1] for point in self.path_points]
        ys = [point[2] for point in self.path_points]
        self.path_line.set_data(xs, ys)
        self.start_line.set_data([xs[0]], [ys[0]])
        self.latest_line.set_data([xs[-1]], [ys[-1]])

        for artist in self.switch_artists:
            artist.remove()
        self.switch_artists = []

        switch_points_by_source = {source: ([], []) for source in COLORS}
        for index, (t, source, source_x, source_y) in enumerate(self.switches, start=1):
            if source_x is not None and source_y is not None:
                x, y = source_x, source_y
            else:
                point = nearest_path(self.path_points, t)
                if point is None:
                    continue
                _, x, y = point
            if x is None or y is None:
                continue
            color = COLORS.get(source, "gold")
            if source not in switch_points_by_source:
                switch_points_by_source[source] = ([], [])
                self.switch_lines[source], = self.ax.plot(
                    [],
                    [],
                    color=color,
                    marker="o",
                    linestyle="None",
                    markersize=8,
                    markeredgecolor="white",
                    markeredgewidth=0.9,
                    label=f"switch to {source}",
                    zorder=6,
                )
            switch_points_by_source[source][0].append(x)
            switch_points_by_source[source][1].append(y)
            self.switch_artists.append(
                self.ax.annotate(
                    f"{index}: {source}",
                    xy=(x, y),
                    xytext=(6, 6),
                    textcoords="offset points",
                    fontsize=8,
                    color=color,
                    weight="bold",
                )
            )

        for source, (switch_xs, switch_ys) in switch_points_by_source.items():
            self.switch_lines[source].set_data(switch_xs, switch_ys)

        self.ax.set_title(f"{self.args.title} | switches: {len(self.switches)}")
        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.axis("equal")
        self.ax.legend(loc="best")
        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def log_topic_diagnostics(self) -> None:
        odom_publishers = self.get_publishers_info_by_topic(self.args.odom_topic) if self.args.odom_topic else []
        pose_publishers = self.get_publishers_info_by_topic(self.args.pose_topic) if self.args.pose_topic else []
        source_publishers = self.get_publishers_info_by_topic(self.args.source_topic)
        publisher_summary = (
            f"publishers: {self.args.odom_topic}={len(odom_publishers)}, "
            f"{self.args.source_topic}={len(source_publishers)}"
        )
        if self.args.pose_topic:
            publisher_summary += f", {self.args.pose_topic}={len(pose_publishers)}"
        self.get_logger().info(publisher_summary)
        for info in odom_publishers:
            self.get_logger().info(
                f"  odom publisher node={info.node_name} ns={info.node_namespace} "
                f"reliability={info.qos_profile.reliability.name} "
                f"durability={info.qos_profile.durability.name}"
            )

    def maybe_log_waiting(self) -> None:
        now = time.monotonic()
        if now - self.last_wait_log_time < 3.0:
            return
        self.last_wait_log_time = now
        elapsed = now - self.start_wall_time
        odom_publishers = self.get_publishers_info_by_topic(self.args.odom_topic) if self.args.odom_topic else []
        pose_publishers = self.get_publishers_info_by_topic(self.args.pose_topic) if self.args.pose_topic else []
        source_publishers = self.get_publishers_info_by_topic(self.args.source_topic)
        self.get_logger().warn(
            f"waiting {elapsed:.1f}s for path samples on {self.args.odom_topic}; "
            f"odom_publishers={len(odom_publishers)} source_publishers={len(source_publishers)}"
        )
        if not pose_publishers and not odom_publishers:
            topics = sorted(name for name, _types in self.get_topic_names_and_types())
            nearby = [name for name in topics if "odom" in name.lower() or "pose" in name.lower() or "localization" in name.lower()]
            if nearby:
                self.get_logger().warn("nearby topics:\n  " + "\n  ".join(nearby))

    def save_if_requested(self) -> None:
        if self.args.output:
            plot(self.path_points, self.source_points, self.args.output, self.args.title)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "bag",
        type=Path,
        nargs="?",
        help="Path to rosbag2 directory, .db3, or .mcap file. Omit with --live.",
    )
    parser.add_argument("--live", action="store_true", help="Subscribe to live ROS topics and update the plot window")
    parser.add_argument(
        "--odom-topic",
        default="/gicp/localization/odom",
        help="Odometry path topic to trace",
    )
    parser.add_argument(
        "--pose-topic",
        default="",
        help="Optional PoseStamped topic to also accept as path samples",
    )
    parser.add_argument(
        "--source-topic",
        default="/gicp/localization/source_status",
        help="MarkerArray topic containing active source status",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output PNG path. In --live mode, saved when you close/interrupt the plot.",
    )
    parser.add_argument("--title", default="Localization Source Switches")
    parser.add_argument("--update-period", type=float, default=0.25, help="Live plot update period in seconds")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.live:
        rclpy.init()
        node = LiveSwitchPlotter(args)
        try:
            while rclpy.ok() and plt.fignum_exists(node.fig.number):
                rclpy.spin_once(node, timeout_sec=0.05)
                plt.pause(0.001)
        except KeyboardInterrupt:
            pass
        finally:
            node.save_if_requested()
            node.destroy_node()
            rclpy.shutdown()
        return 0

    if args.bag is None:
        raise SystemExit("bag path is required unless --live is set")
    if args.output is None:
        args.output = Path("source_switches.png")
    path_points, source_points = read_bag(args.bag, args.odom_topic, args.pose_topic, args.source_topic)
    plot(path_points, source_points, args.output, args.title)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
