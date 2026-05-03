#!/usr/bin/env python3
"""
Extract initial state (V(0) and R_world_imu) for GLIM's URDF-based init from a rosbag.

Reads the first GLIM-IMU message stamp, finds the nearest
/vectornav/velocity_body and /vectornav/imu samples, and prints
the JSON snippet to paste into config_odometry_gpu.json.

Assumes URDF says R_baselink_vectornav = identity AND
R_baselink_imu_bottom = identity (av24.urdf does both). With those
assumptions:
- V_world_imu ≈ V_body from /vectornav/velocity_body (lever-arm term
  ω×r is small at startup; see analysis in chat)
- R_world_imu = R_world_vectornav (since vectornav and imu_bottom
  share base_link rotation = identity)

The vectornav reports orientation in 'earth' frame for /vectornav/imu
(ENU-aligned). We use it directly as the world-frame orientation. The
choice of yaw is gauge-free in GLIM (the GNSS module re-aligns world
to UTM via SVD afterwards), so heading sign conventions don't matter.

Usage:
    python3 extract_init_velocity.py <bag_dir>
        [--imu-topic /gps_bot/imu]
        [--vel-topic /vectornav/velocity_body]
        [--ori-topic /vectornav/imu]
"""

import argparse
import glob
import struct
import sys
from pathlib import Path

try:
    from mcap.reader import make_reader
except ImportError:
    sys.exit("install mcap: pip install --user --break-system-packages mcap")


def parse_imu_stamp(buf):
    o = 4  # CDR encapsulation
    sec, nsec = struct.unpack_from('<II', buf, o)
    return sec + nsec * 1e-9


def parse_twist_with_cov_stamped(buf):
    """geometry_msgs/TwistWithCovarianceStamped (CDR alignment is relative to
    start of CDR data, i.e. byte 4 of the buffer, not absolute offset)."""
    BASE = 4  # encapsulation header
    cdr = 0   # offset within CDR stream
    def take(fmt, sz, align):
        nonlocal cdr
        cdr = (cdr + align - 1) & ~(align - 1)
        v = struct.unpack_from(fmt, buf, BASE + cdr)
        cdr += sz
        return v
    sec, = take('<I', 4, 4)
    nsec, = take('<I', 4, 4)
    slen, = take('<I', 4, 4)
    cdr += slen  # string bytes
    lx, ly, lz, ax, ay, az = take('<6d', 48, 8)
    return sec + nsec * 1e-9, (lx, ly, lz), (ax, ay, az)


def parse_imu_msg(buf):
    """sensor_msgs/Imu (orientation only — we don't need accel here)."""
    BASE = 4
    cdr = 0
    def take(fmt, sz, align):
        nonlocal cdr
        cdr = (cdr + align - 1) & ~(align - 1)
        v = struct.unpack_from(fmt, buf, BASE + cdr)
        cdr += sz
        return v
    sec, = take('<I', 4, 4)
    nsec, = take('<I', 4, 4)
    slen, = take('<I', 4, 4)
    cdr += slen
    qx, qy, qz, qw = take('<4d', 32, 8)
    return sec + nsec * 1e-9, (qx, qy, qz, qw)


def main():
    import math
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_dir')
    ap.add_argument('--imu-topic', default='/gps_bot/imu')
    ap.add_argument('--vel-topic', default='/vectornav/velocity_body')
    ap.add_argument('--ori-topic', default='/vectornav/imu')
    args = ap.parse_args()

    chunks = sorted(glob.glob(str(Path(args.bag_dir) / '*.mcap')))
    if not chunks:
        sys.exit(f'no .mcap files in {args.bag_dir}')

    first_imu_stamp = None
    best_vel = None  # (|dt|, stamp, linear, angular)
    best_ori = None  # (|dt|, stamp, quat)

    for path in chunks:
        with open(path, 'rb') as f:
            reader = make_reader(f)
            for schema, channel, msg in reader.iter_messages(
                    topics=[args.imu_topic, args.vel_topic, args.ori_topic]):
                if channel.topic == args.imu_topic and first_imu_stamp is None:
                    first_imu_stamp = parse_imu_stamp(msg.data)
                elif channel.topic == args.vel_topic and first_imu_stamp is not None:
                    vt, lin, ang = parse_twist_with_cov_stamped(msg.data)
                    dt = abs(vt - first_imu_stamp)
                    if best_vel is None or dt < best_vel[0]:
                        best_vel = (dt, vt, lin, ang)
                elif channel.topic == args.ori_topic and first_imu_stamp is not None:
                    ot, q = parse_imu_msg(msg.data)
                    dt = abs(ot - first_imu_stamp)
                    if best_ori is None or dt < best_ori[0]:
                        best_ori = (dt, ot, q)
                if (best_vel is not None and best_ori is not None and
                        best_vel[1] > first_imu_stamp + 0.5 and
                        best_ori[1] > first_imu_stamp + 0.5):
                    break
            if (best_vel is not None and best_vel[0] < 0.05 and
                    best_ori is not None and best_ori[0] < 0.05):
                break

    if first_imu_stamp is None:
        sys.exit(f'no messages on {args.imu_topic}')
    if best_vel is None:
        sys.exit(f'no messages on {args.vel_topic} near first IMU stamp')
    if best_ori is None:
        sys.exit(f'no messages on {args.ori_topic} near first IMU stamp')

    dvt, vt, (lx, ly, lz), (avx, avy, avz) = best_vel
    dot, ot, (qx, qy, qz, qw) = best_ori

    # Decode quat to roll/pitch/yaw for human sanity check
    sinr = 2 * (qw * qx + qy * qz)
    cosr = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)
    sinp = 2 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1, min(1, sinp)))
    siny = 2 * (qw * qz + qx * qy)
    cosy = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)

    print(f'first {args.imu_topic} stamp: {first_imu_stamp:.6f}')
    print()
    print(f'nearest {args.vel_topic}: stamp={vt:.6f} dt={dvt*1000:.1f}ms')
    print(f'  linear  (m/s):   {lx:+.4f}  {ly:+.4f}  {lz:+.4f}')
    print(f'  angular (rad/s): {avx:+.4f}  {avy:+.4f}  {avz:+.4f}')
    print()
    print(f'nearest {args.ori_topic}: stamp={ot:.6f} dt={dot*1000:.1f}ms')
    print(f'  quaternion (qx, qy, qz, qw): [{qx:+.6f}, {qy:+.6f}, {qz:+.6f}, {qw:+.6f}]')
    print(f'  rpy (deg): roll={math.degrees(roll):+.3f}  '
          f'pitch={math.degrees(pitch):+.3f}  yaw={math.degrees(yaw):+.3f}')
    print()

    # V_world = R_world_imu * V_body. Rotate body-frame velocity into world.
    # (Quat → matrix, Hamilton convention with qw scalar.)
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    R = (
        (1 - 2*(yy+zz), 2*(xy-wz),     2*(xz+wy)),
        (2*(xy+wz),     1 - 2*(xx+zz), 2*(yz-wx)),
        (2*(xz-wy),     2*(yz+wx),     1 - 2*(xx+yy)),
    )
    Vw = (
        R[0][0]*lx + R[0][1]*ly + R[0][2]*lz,
        R[1][0]*lx + R[1][1]*ly + R[1][2]*lz,
        R[2][0]*lx + R[2][1]*ly + R[2][2]*lz,
    )
    print(f'V_world = R_world_imu * V_body = [{Vw[0]:+.4f}, {Vw[1]:+.4f}, {Vw[2]:+.4f}] m/s')
    print(f'  |V_world| = {math.sqrt(sum(v*v for v in Vw)):.4f} m/s '
          f'(should match |V_body| = {math.sqrt(lx*lx+ly*ly+lz*lz):.4f})')
    print()
    print('paste into config_odometry_gpu.json (uncomment URDF init block):')
    print(f'    "init_T_world_imu": [0.0, 0.0, 0.0, '
          f'{qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f}],')
    print(f'    "init_v_world_imu": [{Vw[0]:.4f}, {Vw[1]:.4f}, {Vw[2]:.4f}],')


if __name__ == '__main__':
    main()
