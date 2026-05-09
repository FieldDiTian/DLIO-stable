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

### 6. Things agent reviewers have hallucinated

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
