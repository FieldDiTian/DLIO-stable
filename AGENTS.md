# AGENTS.md — Notes for AI reviewers

This file documents patterns in DLIO++ that **look** like bugs but aren't,
along with the conditions under which they would become bugs. The goal is
to keep automated reviewers from re-flagging the same false positives every
session.

If you are reviewing this codebase: read this file first.

---

## Confirmed non-issues

### 1. `Isotropic::Information(prior_inf_scale.asDiagonal())` is correct (was — superseded)

**File**: `GLIM/glim_ext/modules/mapping/gnss_global/include/glim_ext/gnss_global_module.hpp`
(now uses `Diagonal::Precisions(prior_inf_scale)` after commit `d8b2809`)

**What it looks like**: `Isotropic::Information(...)` doesn't exist in GTSAM.
You'll find no method by that name in `gtsam/linear/NoiseModel.h`. A first
read suggests an API mismatch.

**Why it was actually fine**: in C++, static-method name lookup via inheritance
resolves `Isotropic::Information(M)` to `Gaussian::Information(const Matrix&)`
(the inheritance chain is `Isotropic → Diagonal → Gaussian`). It builds a
Gaussian noise model with the diagonal information matrix — functionally
equivalent to `Diagonal::Precisions(v)` for diagonal arguments. The new form
is preferred for clarity but the old form was not buggy.

---

### 2. `getGtPoseAt` is safe against out-of-order GT odom

**File**: `gicp_localization/src/localization.cc:2365-2401` (function),
`callbackGtOdom` at `:2348-2356` (insertion).

**What it looks like**: the function uses `std::lower_bound` and computes
`u = (stamp - a->stamp) / dt_total` without much obvious defense.

**Why it's actually fine**:
- The buffer is kept monotone non-decreasing by an explicit drop at insertion:
  ```cpp
  if (!gt_odom_buffer_.empty() && s.stamp <= gt_odom_buffer_.back().stamp) return;
  ```
  Out-of-order or duplicate GT messages are discarded. `lower_bound` is
  therefore valid.
- `dt_total <= 0.0` is checked before division (line 2392).
- By construction, the chosen `u ∈ [0, 1]` because `lower_bound` puts the
  query strictly between `a->stamp` and `b->stamp`, so no extrapolation.
- Theoretical "tiny dt_total = 1 picosecond" concerns require GT samples a
  picosecond apart — non-physical for any real GNSS/INS publisher.

---

### 3. `callbackGtOdom` setting `first_opt_done = true` is not skipping initialization

**File**: `gicp_localization/src/localization.cc:2307-2318`

**What it looks like**: `applyInitialPose()` is called and then
`geo.first_opt_done = true` is set unconditionally, which on a casual read
appears to skip the first-time observer initialization in
`performLocalization()` at line 2106.

**Why it's actually fine**: `applyInitialPose()` itself zero-initializes the
observer state inside `geo.mtx` (lines 1068-1083): `state.v.* = 0`,
`state.b.* = 0`, `geo.prev_p/q = (initial pose)`, `geo.prev_vel = 0`. So by
the time `first_opt_done` is flipped, the state is already in the same
condition the `if (!geo.first_opt_done)` block at 2106-2123 would have set
it to. The first scan goes directly to `updateState()` operating on a
properly-initialized observer. The flag flip is correct.

---

### 4. The hessian-condition rejection disjunction is correct

**File**: `gicp_localization/src/localization.cc:2030-2041`

**What it looks like**: a 4-clause OR with three "warn-enabled AND warn-crossed"
sub-clauses plus a fourth "all warns disabled" sub-clause. Easy to convince
yourself the legacy fallback always fires, or that any single threshold
disables the others.

**Why it's actually fine**: trace the truth table.
- The fourth sub-clause `(fitness_warn ≤ 0 && trans_warn ≤ 0 && rot_warn ≤ 0)`
  fires only when **all three** warn thresholds are disabled — that's the
  intentional "legacy hessian-alone" fallback documented in the comment at
  line 2049.
- With YAML defaults (all three thresholds positive: `0.15`, `1.0 m`, `1.5°`),
  the fourth clause is unreachable by design. Rejection then requires hessian
  degenerate AND at least one of the three slide signals crossed.
- A "valid" scan (low fitness, small corrections) never satisfies the
  disjunction even when hessian is high.

The block matches its comment exactly. Don't refactor unless asked.

---

### 5. The URDF rotation between IMU and GNSS antenna is discarded — and that's fine on AV-24

**File**: `GLIM/glim_ext/modules/mapping/gnss_global/include/glim_ext/gnss_global_module.hpp:107`

**What it looks like**: `t_imu_gnss = T_imu_gnss.translation()` throws away
the rotation part of an SE(3) transform. Looks like a missed correction.

**Why it's actually fine on AV-24**: in `av24.urdf`, both `novatel_a_joint`
(the IMU per `urdf_imu_frame: "novatel_a"`) and `gps_antenna_right_joint`
(the antenna) are origin-only — no `rpy` attributes — and share the same
parent link `rear_axle_middle`. So `R_imu→antenna = identity`, and the
translation is the same vector regardless of which of those two frames you
express it in.

**Watch condition**: if a future URDF adds an `rpy` to either joint, or the
`urdf_imu_frame` is switched to a non-axis-aligned IMU, the lever-arm
correction will become biased. A startup warning was added in commit
`622271f` to flag this — heed it if you see it in the logs.

---

### 6. GNSS lever-arm double-compensation is not a live risk in the current config

**Files**:
- `GLIM/glim/config/config_ros.json` (`extension_modules` list)
- `GLIM/glim/config/config_odometry_ins.json` (`urdf_ins_frame`)
- `GLIM/glim_ext/modules/mapping/gnss_global/include/glim_ext/gnss_global_module.hpp:361-368`

**What it looks like**: the GNSS extension subtracts `R_world_imu * t_imu_gnss`
from the reported GNSS position. If the Novatel receiver is also configured
with `LEVERARMCONFIG`, both firmware and software would compensate, biasing
the GNSS prior by ~0.6 m horizontal on AV-24 (the `novatel_a` →
`gps_antenna_right` offset).

**Why it's actually fine on AV-24 today**:
- `libgnss_global.so` is **commented out** in `config_ros.json`'s
  `extension_modules` block. The only site that applies the subtract is
  never loaded, so mapping does not double-compensate.
- The INS-driven odometry frontend (`libodometry_estimation_ins.so`) computes
  its own `T_imu_ins` from URDF but resolves to identity in the current
  config: `urdf_imu_frame: "novatel_a"` and `urdf_ins_frame: "novatel_a"`.
- The localization node (`gicp_localization`) does not apply a GNSS
  lever-arm correction at all. As of the single-source NA design, it
  consumes `/gps_na/filtered_odom` (NA INS pre-VKS, naturally at
  NA_IMU_Frame) as-is. The lever arm is handled inside the NovAtel
  firmware. `base_frame`, `imu_frame`, and the gt_odom source are all
  at `novatel_a`, so the in-code TF lookups (`baselink2imu_T`,
  `T_base_gtbody_`) resolve to identity. See
  `gicp_localization/docs/GICP_GNSS_IMU_bug_report.pdf` for the
  architectural alternatives that were considered.

**Watch condition**: if `libgnss_global.so` is uncommented, or
`urdf_ins_frame` is changed to a different link than `urdf_imu_frame`,
verify the Novatel `LEVERARMCONFIG` state before merging — software
compensation must only be on when the firmware is off, and vice versa.
Setting `urdf_gnss_frame: ""` in `config_gnss_global.json` hard-disables
the software side even if the extension is re-enabled.

---

### 7. The GT-recovery RTK gating lives upstream, not in our code

**File**: `gicp_localization/src/localization.cc:2289-2370` (`callbackGtOdom`).

**What it looks like**: `callbackGtOdom` ingests every odometry message it
receives and pushes it into the buffer with no RTK / fix-status check.
A reviewer might flag this as a missing guard — snap-back could fire from
a degraded GNSS fix.

**Why it's mostly fine on AV-24** (under the single-source NA design):
`/gps_na/filtered_odom` is published by race_common's `novatel_interface`
node, which applies its own quality gates (`sensors.position_rms`,
`sensors.heading_stdev_max`, `sensors.activation_speed`, and the
`bypass_checks` flag in `novatel_interface.param.yaml`). Combined with
the 0.1 s `gt_odom/max_dt` window, a stalled or rejected upstream
publisher cleanly defers snap-back (logged as
`deferring snap — no GT sample within max_dt…`) rather than firing on
stale data.

**RTK-contract gate now enforced in code**: the previous design relied
on `/localization/global/odom` (voted INS output) stopping publication
entirely when RTK is not fixed. The new single-source NA design
subscribes to `/gps_na/filtered_odom` directly, which can keep publishing
through RTK degradations. The localization node now subscribes to the
NovAtel `BESTGNSSPOS` topic (default `/novatel_a/bestgnsspos`,
remappable as `rtk_status`) and rejects every `gt_odom` sample whose
cached `pos_type` is not in the RTK-fixed set (`NARROW_INT=50` or
`INS_RTKFIXED=56`; plus `NARROW_FLOAT=34` / `INS_RTKFLOAT=55` when
`localization/rtk_gate/allow_float=true`). Stale status (older than
`localization/rtk_gate/max_status_age`, default 2 s) is treated as
not-fixed. See `callbackRtkStatus` and the RTK-gate block at the top of
`callbackGtOdom`. Reviewers should not flag the missing fix-status guard
in the legacy code path — it now exists explicitly.

**Watch condition**: if the RTK gate is disabled (`rtk_gate/enable=false`)
or the `rtk_status` topic is misrouted, the joint failure mode to keep
in mind is a low-feature LiDAR stretch coinciding with an RTK
degradation: GICP can't recover geometrically and the snap pulls toward
a degraded GNSS fix.

---

### 8. Things agent reviewers have hallucinated

These were flagged by previous agent runs but **do not exist in the code**.
If you find yourself about to flag one of these, double-check first.

- **"`swapSourceAndTarget` clears the IMU buffer."** False. `swapSourceAndTarget`
  is in `nano_gicp.cc:96-104`; it swaps PCL source/target clouds and clears
  `correspondences_` / `sq_distances_`. It has no relationship to the
  localization node's `imu_buffer`.

- **"`maybeSnapPoseToGT` asserts that `pose_mutex` is held by the caller."**
  False. There is no such assertion in the function (`localization.cc:2404+`).
  The call graph happens to be safe today because the caller is on the LiDAR
  thread, but no compile-time or runtime guard enforces this — flag the
  *real* concern (no explicit lock contract) rather than a fabricated assert.

- **"`Isotropic::Information(asDiagonal())` causes a dimension mismatch /
  silent isotropic noise model."** False — see #1 above.

- **"Aux LiDAR `time_field` precision loss for FLOAT32 is dramatic."** Reality:
  the `+= static_cast<float>(dt)` call works correctly when both `val` and
  `dt` are in seconds (Velodyne convention). The genuine concern is that the
  unit semantic of FLOAT32 time fields is sensor-dependent and undocumented;
  describe that, not a fabricated precision-loss number.

---

## Real issues that are intentional tradeoffs

### A. `localization.yaml` defaults assume default sensors

`use_sim_time: true`, the Luminar primary sensor type, the multi-LiDAR concat
list, and the GT recovery defaults are all calibrated for the AV-24
Cybertruck rig running rosbag-style replay or live with `/clock`. Other rigs
should override via launch args, not by editing the YAML.

### B. The "race-day workaround" commits are a system, not isolated patches

`4a594a9` (slide-along-unconstrained-axis catcher), `83b48a4` (GT-driven
recovery), `d3ec851` (rejection hardening), and `4b74d30` (live-checking
switch to GT) all interact through `performLocalization()` to produce a
robust-but-tangled rejection/recovery state machine. Don't review them in
isolation. A future refactor should consolidate them, but for now treat the
ensemble as load-bearing.

### C. Maps live outside the repo

`localization/map_path` defaults to `dlio_maps/may_4_putnam.pcd` (relative)
but no map is checked in. Production runs override the path via the
working tree or a launch arg pointing at external storage. The `dlio_maps/`
directory does not exist in tree by design.

---

## How to use this file

When reviewing this codebase:
1. Skim this file before flagging an issue.
2. If you find a candidate problem, verify the actual code state matches
   your description before reporting — agent runs in this repo have a
   recurring pattern of fabricated line numbers and inverted conditions.
3. If you confirm a previously-documented non-issue has become a real
   issue (e.g., URDF rpy added per #5 watch-condition), update this file
   in the same commit as the fix.

## Imported Claude Cowork project instructions

Improve the GICP localization algorithm by integrating with a race car platform and two integrated GNSS systems, one for NovAtel, one for vectorNav
