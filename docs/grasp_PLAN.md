# AMENDMENT (2026-07-14) — replace THE CLOSE with the contact-stop ladder

**Scope: the gripper close, and nothing else.** The MTC task graph, the `GetGrasps` seam, the
analytic/GPD/GraspGen servers, the nav stack and every gate below are unchanged and still govern.

## Context — why

The pick has never satisfied its gate (MTC success ∧ mine lifted >0.06 m ∧ base displacement
<0.03 m). The Husky gets levered/thrown ("the roll"). Root cause, now settled and *quantified*:

The 2F-140 cannot exert grip force in Gazebo — every mimic joint is driven by
`robotiq_gazebo`'s `MimicJointPlugin` via `SetPosition()` (a kinematic teleport), and
`right_outer_knuckle_joint` is itself a mimic, so the whole right finger is kinematic. It is
forced to its commanded angle *regardless of contact*. It therefore never stalls — it
**penetrates**, and the penetration impulses are what throw the 46 kg base.

`docs/UGV定点抓取雷达测试方法总结 (2).md` (from teammates) reaches the same diagnosis and supplies
a pad-gap-vs-`finger_joint` table. **I reproduced that table exactly from our own URDF by pure FK**
(mimic multipliers read from the URDF, no hand-guessed signs) — all six rows to three decimals:

| `finger_joint` | 0.480 | 0.490 | 0.495 | 0.498 | 0.500 | 0.513 | **0.58** |
|---|---|---|---|---|---|---|---|
| pad gap (mm) | 43.456 | 41.503 | 40.525 | 39.938 | 39.546 | 36.996 | **23.762** |

Which indicts our own config. `husky_ur5.srdf:49` claims "measured: q=0.537 → 40 mm". **False** —
40 mm is at q≈0.495. So our `closed = 0.58` drives a **23.76 mm gap onto a 40 mm block: 16.2 mm of
interpenetration, 8 mm per pad.** The teammates' doc reports that a *3 mm* overclose (q=0.513) was
already enough to eject the mine and blow up the solver. **Ours is 5.4× that. That is the roll.**

## The fix (their method, adopted)

Never command a gap narrower than the object. Close in stages from **fully open**, poll for the
contact event, **stop on contact**, then squeeze 0.003 rad. `gazebo_grasp_fix` — already wired in
`husky_ur5.urdf.xacro:191` and permissive (`grip_count_threshold=1`, `forces_angle_tolerance=100`) —
then welds, which is the only thing that can hold the mine. It has never once fired here, plausibly
*because* we always penetrated the block by 16 mm and ejected the mine before a stable contact could register.

Two facts that make this land cleanly:

- **`gazebo_grasp_plugin_ros/grasp_event_republisher` is already vendored** (`src/gazebo-pkgs/`) and
  has simply never been launched. `grasp_fix` publishes attach events on a *Gazebo transport* topic
  (`~/grasp_events`, protobuf), not ROS; this node bridges it to `GazeboGraspEvent{arm, object,
  attached}`. That is the teammates' `mine_grasp_event_republisher`. **The event IS the weld** —
  `grasp_fix` publishes `attached=true` at the moment it creates the fixed joint, so it cannot be
  used as a pure contact sensor. Decided: `grasp_fix` is the sole holder; **no `link_attacher`
  fallback** (the package stays in-tree, unused, and the world plugin stays loaded but idle).
- **Yaw is safe here.** A 40 mm square yawed θ presents `40(cosθ+sinθ)` mm, so the ladder's first
  rung (43.46 mm) only clears |θ| ≲ 5°. But `analytic_grasp_server.py:124` already emits **four yaws
  at 90° spacing**, and a square has 90° symmetry → all four present exactly 40.0 mm. Decided: use
  the teammates' ladder **verbatim**. The existing `block_yaw` warning (a 45°-off yaw is a corner
  grasp) already covers the residual risk.

## Changes

**1. `src/husky_ur5_moveit_config/config/husky_ur5.srdf`**
- Delete `pregrasp` (0.45 → 49.3 mm gap): it exists only to shorten the final close, and it is what
  rams a yawed block on the way down. The approach now descends fully open (128.5 mm), which clears
  any yaw.
- `closed`: 0.58 → **0.500** (39.546 mm — the ladder's last rung, so the state stays truthful).
- Replace the comment's invented gap curve with the FK-verified table above.

**2. `src/grasp_mtc/scripts/grasp_task.py`** — the substance. The architecture *already* has the
right seam: `build_reach_task()` ends **at** the grasp pose with nothing touched, then a hold step,
then `build_lift_task()`. Only the hold step changes.
- `build_reach_task()`: `pregrasp` → `open`; rename the stage to `open gripper` and keep
  `gp.setMonitoredStage(task['open gripper'])` (GeneratePose must still monitor the *last gripper
  stage*, or `Connect` — which plans the arm group only — sees mismatched gripper joints and returns
  0 solutions).
- **Replace `_weld()` with `_close_on_contact()`**, driving `/gripper_controller/follow_joint_trajectory`
  (action, not the topic — we need per-rung arrival, and must accept `GOAL_TOLERANCE_VIOLATED`):
  - ladder `[0.480, 0.490, 0.495, 0.498, 0.499, 0.500]`, speed ≤ 0.06 rad/s, ≥0.60 s per rung;
  - after each rung, poll 0.35 s at 50 Hz for `attached`; on contact, stop the ladder immediately;
  - squeeze to `min(0.500, q_contact + 0.003)` at 0.01 rad/s, using the **measured** contact q;
  - require `attached` **continuously ≥1.0 s within 2.5 s**; any dropout resets the timer.
- Subscribe to the republished `grasp_events`; set `self.gazebo_attached` when `object` starts with
  `landmine`.
- **Empty-grasp detection**: ladder ran to 0.500 with no attach → `EMPTY_GRASP`; attach never came
  but the jaws stalled short → `GAZEBO_ATTACH_FAILED`. Report it; **do not silently substitute a weld.**
- After the lift, assert `attached` is still true (cheap hold check).
- Delete `_weld`/`_unweld` and the `gazebo_link_attacher` import. Release is now just **opening the
  jaws** — `grasp_fix` detaches on its own (`release_tolerance = 0.005`). `build_place_task()` →
  `build_release_task()` already open in the right order; `_abort_lost_grasp()` already opens first.

**3. `src/grasp_mtc/scripts/analytic_grasp_server.py`** — `PAD_ADVANCE_M: 0.024 → 0.0215`. This is
part of the close, not the detector: the aim height pre-compensates how far the pads travel *down*
the approach axis while closing, and closing now starts from open. FK: `grasp_tcp` is at 0.177 from
the palm and the pad centre at q=0 is **also 0.1770** (the TCP *is* the pad plane when open); at
q=0.495 the pads sit at 0.1985 → **21.5 mm** of advance. Pads then rest 20 mm below the top face
(z≈0.065), lower edge 10 mm clear of the disc — matching the teammates' "8 mm above the detonator
centre". The old "pads drag down the block faces and pry `finger_joint` negative" failure is void:
the gap only reaches 40 mm at q≈0.495, by which point the pads are already at final depth.

**4. `src/grasp_mtc/launch/manipulation.launch`** — launch
`gazebo_grasp_plugin_ros/grasp_event_republisher`.

**5. `src/grasp_mtc/package.xml`** — add `gazebo_grasp_plugin_ros`, `control_msgs`; drop
`gazebo_link_attacher`.

`src/mobile_manipulator/config/ur5_controllers.yaml` already has `finger_joint: {trajectory: 0.0}`
(path tolerance disabled) — required, and unchanged.

## Verification

`catkin build`, then the existing headless harness (`round_headless.sh` → `trials.sh` → `validate2.py`,
`trace3.py`), **10 trials**, gate per trial unchanged:

> **MTC success ∧ peak mine z > 0.06 m ∧ max base displacement < 0.03 m**

Plus two cross-checks this change makes possible:
- **Log the contact q per trial.** At yaw 0 it must land at **0.495–0.500** (gap ≈ 40 mm). The
  teammates saw 0.479–0.489, i.e. a ~5°-yawed mine — so a match on *gap* rather than on *q* is the
  correct agreement, and confirms the FK.
- **Whether `grasp_fix` fires at all.** If `attached` never arrives, that is the decisive result and
  gets reported as-is (`EMPTY_GRASP` / `GAZEBO_ATTACH_FAILED`) — not papered over with a weld.

Report the full trial table honestly, including failures. Then offer to restore the mission stack
(`bash airground_takeoff.sh`).

---

# STEP 3 — Grasping the landmine detonator in the air-ground outdoor mission

*(Governing plan for everything except the close — see the amendment above. Phase 4.6's single
`MoveTo(gripper, "closed")` is the one stage the amendment replaces.)*

### Status (2026-07-14) — branch `feature/landmine-grasp`

| Phase | State |
|---|---|
| 1 — nav stack, single elevation owner | **done** |
| 2 — landmine in world, reachability, colour detector, tour hook | **done.** Gate 2 answered: a top-down grasp is reachable only for **x ∈ [0.60, 1.05] m** from `base_link`, hard cliff at 0.55 — so `reach_tolerance = 0.6` parks right on the cliff and a distinct `grasp_standoff` (0.75) was needed, exactly as the risk section predicted |
| 3 — `move_group`, `grasp_tcp`, SRDF, magic scalars deleted | **done** |
| 4 — MTC pick, analytic grasp | **blocked on the close — this is what the amendment unblocks.** Reach/lift/lower/release all plan and execute; no trial has yet passed the gate |
| 5 — GPD, GraspGen A/B | not started; unchanged |

The remaining backlog outside the close, unchanged and untouched by the amendment: wire `landmine.onnx`
into the TensorRT `trt_yolo_node.cc`; the ≥20-trial success-rate campaign (Gate 4); `gpd_grasp_server.py`;
and the `ARRIVED → GRASP` standoff handoff in `ugv_target_tour.py`.

---

## Context

The live system is the air-ground mission launched by **`airground_takeoff.sh`**: egocentric, `odom`-rooted,
DLIO-owned, no `map` frame, no `move_base`.

**Mission today:** UAV (PX4 + EGO) flies and scans → elevation map → UGV (DLIO + CMU local_planner) tours to
each detected target → **and drives away.** `ugv_target_tour.py` runs `COLLECT → TOUR → DONE`; on arrival it
logs `reached N/M` and immediately calls `_advance()`. There is no grasp phase, and **`move_group` is never
launched** — the arm is spawned stowed (`spawn_outdoor_city.launch:58-63`) and never commanded again. The
`ur5_arm_controller` / `gripper_controller` *are* spawned (`:80-81`), so the hardware is ready and idle.

**Task:** the UGV arrives at a detected landmine, stops, unstows the arm, and **grasps the detonator**.

### The target: `gazebo_models/landmine` (already exists, purpose-built)

| Part | Geometry | Color | Role |
|---|---|---|---|
| Disc body | red cylinder, ⌀136 mm × 25 mm | `diffuse 0.85 0.1 0.1` | mine silhouette; context cue |
| **Detonator** | **yellow box, 40×40×60 mm** | `diffuse 0.95 0.85 0.0` | **the grasp feature** |

Single rigid link, mass 0.3 kg, `mu = 1.0/1.2`, `kp=1e6`. The SDF's own comment prescribes the grasp:
*"Grasp the 40 mm detonator block **top-down**: closes to 40 mm, 100 mm of 2F-140 stroke margin."*
Detonator top sits at **85 mm above ground**. Friction and inertia are already tuned — do not re-tune them.

**This is a part-level grasp** (the detonator, not the mine) — cheap because the part is **color-coded**: an
HSV threshold on the yellow block is an unambiguous segmentation, with the red disc available to disambiguate.

### Decisions taken
- **Executor is MTC.** Verified: MTC Python bindings are installed (`pymoveit_mtc` v0.1.3)
  and `core.Generator` / `core.MonitoringGenerator` **are subclassable from Python**.
- **Mock the detector with color** for now; the real UAV detector stays a separate work item.
- **Nav stack upgraded** in `airground_takeoff.sh` to `cost_source:=elevation`, `global_planner:=far`.
- **Scope runs through GraspGen, gated on checkpoint verification.**

---

## Invariants

1. **Do not touch the navigation chain.** DLIO stays the sole `odom`→`base_link` publisher; no `map` frame;
   `/registered_scan`, `/terrain_map`, `/state_estimation` contracts unchanged.
   MoveIt is safe: the SRDF has **no `virtual_joint`**, so `move_group` plans in the URDF root
   (`base_link`) — already robot-relative — and introduces no `map` frame. `move_group.launch` defaults
   `load_robot_description:=false`, so it won't clobber the spawner's URDF.
2. **Do not re-plumb Gazebo grasp physics — fix the COMMAND instead.** The mimic joints, the URDF, the
   controller interfaces and the landmine's friction/inertia stay untouched, because the defect is not in
   any of them — it is the 16 mm over-close we were commanding into a 40 mm block. No effort-interface
   migration; no re-tuning the mine. **The only new plumbing is launching a node that already exists**
   (`grasp_event_republisher`), so we can *observe* what `grasp_fix` does.
3. **Exactly one elevation-map owner.**
4. **Exactly one octomap owner** — MoveIt's `sensors_3d` plugin, fed by the **wrist RealSense**. Keep
   strictly separate from `robot_body_filter` → `/registered_scan` → nav. Do not merge them.
5. **UGV stationary during manipulation** — the CMU planner must idle, not nudge the base.
6. **Detector swappable behind one interface**: `grasp_source:=analytic|gpd|graspgen` changes no MTC code.
7. Feature branch; each phase independently revertable.

---

## PHASE 1 — Nav stack upgrade (and the node collision it would otherwise cause)

**Goal:** `airground_takeoff.sh` runs the current nav stack: `nav:=cmu`, `cost_source:=elevation`,
`global_planner:=far` (per `egocentric_nav.launch`).

- **1.1 Resolve the duplicate `elevation_mapping` node.** **Both** launch files start a node literally
  named `elevation_mapping` in the global namespace:
  - `cmu_planner.launch:79` (`elevation_mapping_ugv.yaml`, + `elevation_mapping_uav_prior.yaml` if
    `uav_prior`) — only when `cost_source:=elevation`
  - `airground_egocentric.launch:123` (`elevation_mapping_uav_live.yaml`) — **unconditionally**

  Running both = name collision = roslaunch silently kills one. **Flipping `cost_source` to `elevation`
  without fixing this will appear to work and quietly destroy one of the two maps.**

  **Resolution:** **one map, UGV-owned, UAV fused in as a second input source.**
  - Add `start_elevation` (default `true`) to `airground_egocentric.launch`, guarding its `elevation_mapping`
    node. `airground_takeoff.sh` passes `start_elevation:=false`; it keeps UAV mapping + anchor + gate.
  - `cmu_planner.launch` owns the map: `cost_source:=elevation uav_prior:=true map_size:=120
    map_resolution:=0.35`.
- **1.2 `enable_gate` — test both.** `egocentric_nav.launch:60-66` says `uav_prior` needs
  `airground_egocentric.launch` running alongside (`enable_gate:=true`), but
  `airground_egocentric.launch:45-51` defaults it **off** ("in practice it performs poorly"). Try `false`
  first; fall back to `true` if the map is polluted by ground-level UAV returns during takeoff.
- **1.3 FAR.** `cmu_planner.launch:27` warns: **never** run `terrain_analysis_ext` alongside `far` — both
  publish `/way_point`. The launch already guards this; confirm at runtime.

**Gate 1:** `airground_takeoff.sh` brings up the UAV + UGV with **exactly one** `elevation_mapping` node
(`rosnode list`), FAR owns `/way_point`, and a hand-published `/ugv/goal` still drives the UGV.

---

## PHASE 2 — Make the mission graspable, and answer the reachability question

- **2.1 Put the landmine in the world.** Add it to `outdoor_city.world` **directly** — not spawned at
  runtime: `elevation_mapping` fuses rather than replaces, so a model introduced after the robot has mapped
  that ground as flat is rejected as an outlier and never enters the map.

- **2.2 REACHABILITY GATE — before any grasp code.**
  The tour parks the UGV within `reach_tolerance = 0.6 m` (`ugv_target_tour.py:60`). The detonator's top is
  **85 mm above ground**, and a **top-down** grasp means the 2F-140 (~160 mm long) hangs below a wrist
  pointing straight down — so the wrist must sit ≳250 mm above ground, reaching *down and out*, from a UR5
  mounted on the Husky roof.
  - Measured: sample IK for the prescribed top-down TCP pose over a grid of ground positions around the base;
    find the reachable annulus.
  - **Result:** fails at 0.6 m — fixed upstream with a distinct **grasp standoff** (separate from
    `reach_tolerance`), parking the UGV at a reachable radius.

- **2.3 Color mock detector** → `scripts/landmine_color_detector.py`. HSV threshold on the **yellow**
  detonator (`0.95 0.85 0.0`), with the **red** disc (`0.85 0.1 0.1`) as a context/disambiguation cue;
  centroid → depth → 3-D point; publish `WorldTarget` (`class_name="landmine"`, `PointStamped`) on
  `/detected_targets`. Runs on the **UGV wrist camera** for the grasp loop.

- **2.4 Grasp hook in the tour.** `ugv_target_tour._loop()` currently calls `_advance()` the moment
  `d < reach_tolerance`. Insert an **`ARRIVED → GRASP`** state before it, behind a `~grasp_on_arrival` param
  (default **false**, so nav-only runs are byte-for-byte unaffected). Advance on grasp success *or* failure.

**Gate 2:** a landmine sits in the world; the color detector publishes a `WorldTarget`; the tour drives to it
and **stops**; and you have a definitive answer on whether the UR5 can reach the detonator from where it parks.

---

## PHASE 3 — MoveIt into the mission + the TCP frame

- **3.1 Launch `move_group`** from the air-ground path (`load_robot_description:=false`).
  *Verify:* `view_frames` shows **no `map` frame**; planning frame is `base_link`; DLIO still sole owner of
  `odom→base_link`.

- **3.2 Stow / unstow as SRDF named states.** The SRDF `ready` state stores **`shoulder_lift = 4.683`**
  deliberately (the +2π branch of the target pose) — targeting the naive `-1.60` value makes the planner
  rotate 279° the long way, sweeping the arm straight through the Husky chassis.

- **3.3 The TCP frame — the root cause.** A massless, geometry-less fixed link **`grasp_tcp`** is a child of
  **`robotiq_arg2f_base_link`**. **Convention: `+Z` = approach (out of the palm, between the fingers), `+X` =
  finger-closing (GPD's `binormal`), `+Y` = GPD's `axis`** — making GPD's `R = [approach binormal axis]` a
  pure column permutation. FK-derived, not tuned: the jaw center-line is the midpoint of the two
  `*_inner_finger_pad` links, verified 0.04 mm from that midpoint.

- **3.4 SRDF.** `<end_effector>` and `<group_state>` `open` (`finger_joint=0.0`) / `closed`. Renamed
  `hand_e_gripper` → `gripper`.

- **3.5 Deleted `GPD_TOOL_OFFSET` and `FINGER_REACH`.** With `ik_frame = grasp_tcp`, MTC does this work.

- **3.6 Octomap** from a self-filtered wrist-RealSense cloud into the already-configured
  `PointCloudOctomapUpdater` (`/scene_filtered_cloud`).

**Gate 3:** arm unstows/restows on command; `grasp_tcp` sits between the jaws in RViz; both magic scalars gone;
nav unaffected.

---

## PHASE 4 — MTC pick, with an **analytic** grasp first

**The key ordering choice.** The detonator's grasp is *prescribed* by its own SDF (top-down on a 40 mm box).
So prove the mission end-to-end with an **analytic top-down grasp** before introducing any learned detector.
This decouples *"does the mission work"* from *"does the detector work"* — and the `GetGrasps` seam makes the
later swap a one-line launch arg.

- **4.1 The seam:** package `src/grasp_mtc/`, `srv/GetGrasps.srv` — request: target `PoseStamped` +
  segmented `PointCloud2`; response: ranked `PoseStamped[]` (**TCP poses**) + `float32[] width` +
  `float32[] score`. `grasp_source:=analytic|gpd|graspgen` swaps **only the server behind it**.
- **4.2 `analytic_grasp_server.py`** — top-down TCP pose over the color-detected detonator centroid,
  `width = 0.04`. No learning, no cloud. This is the pipeline's ground truth.
- **4.3 The generator stage.** `core.MonitoringGenerator` is Python-subclassable (verified). `GraspGenerator`
  calls `GetGrasps`, spawning one `InterfaceState` per candidate. Replaces the unavailable
  `GenerateDeepGraspPose`.
- **4.4 Stage graph.** The pick is NOT one open-loop MTC task. `task.execute()` returns `None` in
  pymoveit_mtc 0.1.3, so a mid-task trajectory abort is **invisible from Python**. **The close cannot be an
  MTC stage at all** (see 4.6). The graph is therefore split into tasks with physical checks between them:
  ```
  reach_landmine  (MTC)
    CurrentState                                           # scene already holds the landmine
      → MoveTo(arm, "ready")                               # unstow, via the +2π shoulder_lift branch
      → MoveTo(gripper, "open")                            # FULLY open (128.5 mm) — see amendment
      → Connect(ur5_arm)
      → [pick] MoveRelative: approach (Cartesian, grasp_tcp +Z, back-propagated)
               GeneratePose → ComputeIK(ik_frame="grasp_tcp")   ← the seam; no scalar offsets
               allowCollisions(landmine, gripper_links + ground)
    ENDS AT THE GRASP POSE, jaws open, nothing touched.

  == THE CLOSE — the contact-stop ladder, outside MTC (amendment §Changes 2) ==
     ladder → stop on grasp_fix `attached` → squeeze 0.003 rad → confirm ≥1.0 s
     EMPTY_GRASP / GAZEBO_ATTACH_FAILED reported, never papered over.

  lift_landmine   (MTC)   allowCollisions → attachObject(landmine → grasp_tcp) → MoveRelative: lift
                          (Cartesian, along ODOM +Z — see risks; NOT base +Z)

  == LIFT CHECK (grasp_tcp height; `attached` still true) ==
     failed → open the jaws where they are, purge the scene, retreat UPWARD, restow. NO descent.

  lower_landmine  (MTC)   MoveRelative: lower, capped at lift.min so it can never undershoot
  release_landmine(MTC)   MoveTo(gripper,"open") → grasp_fix releases → detach → retreat up → stow
  ```
  The landmine is **one rigid link** (disc + detonator). The gripper grasps the detonator but **lifts the
  whole 0.3 kg mine** — and the disc is a 136 mm collision body directly beneath the jaws. `allowCollisions`
  and `touch_links` must account for the *disc*, not just the detonator, or approach/lift won't plan.
  The mine must also be allowed to touch `ground`, or it reads as permanently in-collision and every
  `ComputeIK` returns zero solutions.
- **4.5 `touch_links` — complete list** (incomplete lists are the #1 cause of "attach succeeded but lift
  won't plan"):
  ```
  robotiq_arg2f_base_link,
  left_inner_finger,  left_inner_finger_pad,  left_inner_knuckle,  left_outer_finger,  left_outer_knuckle,
  right_inner_finger, right_inner_finger_pad, right_inner_knuckle, right_outer_finger, right_outer_knuckle
  ```
- **4.6 Gripper stages are `MoveTo` + named states — EXCEPT the close.** There is **no `GripperCommand`
  action** — the gripper is a `position_controllers/JointTrajectoryController` on `finger_joint`, mapped as
  `FollowJointTrajectory`. Do **not** use `SimpleGrasp`/`Pick`/`Place` stages, which assume a gripper action.
  *`open` stays a `MoveTo` named state.*
  **The close is not an MTC stage.** MTC plans, then executes: it cannot stop a trajectory partway on a
  sensor event, and stopping on contact is the entire fix (amendment). `MoveTo(gripper, "closed")` is
  therefore replaced by `_close_on_contact()`, a feedback loop that runs **between** `reach_landmine` and
  `lift_landmine` and drives the controller's `FollowJointTrajectory` action directly, one rung at a time.
  The SRDF keeps a `closed` state (now **0.500**, the ladder's last rung) so the named state stays truthful,
  but nothing plans to it.
- **4.7 Dual-attach sync.** MoveIt's `attachObject` and `gazebo_grasp_fix`'s physical attach are independent
  (grasp_fix on contact, MTC at the attach stage). The amendment makes agreement verifiable for the first
  time: `grasp_fix` publishes only on a **Gazebo transport** topic, so its attach was previously unobservable
  from ROS. Launching the already-vendored `gazebo_grasp_plugin_ros/grasp_event_republisher` bridges it, and
  the pick now *gates* on that event rather than assuming it.
  - **`grasp_fix` is the sole physical holder.** No `link_attacher` weld, no fallback: if it declines to
    fire, that is the result, and it gets reported (`EMPTY_GRASP` / `GAZEBO_ATTACH_FAILED`).
  - *Note, not a change:* `gazebo_grasp_fix` watches `left/right_inner_finger`, but the real contact surfaces
    are the separate `*_inner_finger_pad` links. Gazebo **lumps** those fixed-joint children into the finger
    links (confirmed in the flattened SDF: `left_inner_finger_pad_collision_1` lives on
    `left_inner_finger`), so the pad contacts *are* attributed to the watched links.

**Gate 4:** full loop — UAV maps → tour drives to the landmine → UGV stops → arm unstows → **analytic
top-down grasp lifts the mine by its detonator** → arm restows → tour continues. ≥20 trials, success rate
recorded. **This is the deliverable.**

---

## PHASE 5 — GPD, then GraspGen (A/B)

Only once Phase 4 passes. Both drop in behind `GetGrasps` with **zero MTC changes** — that's what the seam
buys, and the analytic baseline is the control.

- **5.1 GPD.** Launch it (`husky_ur5_gpd.launch` — built, calibrated, never started). Adapter:
  `TCP = position + approach · d` (GPD `position` = hand *bottom* center; `d` ≈ `hand_depth/2`, from
  `hand_depth=0.05`, `init_bite=0.01` in `husky_ur5_gpd.cfg`); orientation = `[binormal, axis, approach]`
  columns.
  *Verify:* overlay adapter TCP markers on GPD's own `plot_grasps` markers (already enabled,
  `husky_ur5_gpd.launch:18`). They must coincide. **And compare against the analytic grasp** — a strong,
  free ground truth.
- **5.2 GraspGen — verify BEFORE building.** Confirm NVIDIA's **Robotiq-2F-140 checkpoint is actually
  publicly released** and the license permits this use. **If not, stop and stay on GPD.** Hard gate. Then
  write a Noetic node (Python 3, so PyTorch loads) advertising the same `GetGrasps` service.
- **5.3 A/B.** Same object, scene, executor, gripper. Report success rate · latency · #candidates surviving
  IK/collision filtering · failure taxonomy (perception / kinematics / slip), **with the analytic grasp as
  the control arm.**

**Gate 5:** `grasp_source` swaps cleanly across all three; A/B table recorded.

---

## Critical files

- **Modify:** `airground_takeoff.sh` (nav stack + `move_group` + detector) ·
  `src/mobile_manipulator/launch/airground_egocentric.launch` (`start_elevation` guard) ·
  `src/mobile_manipulator/launch/cmu_planner.launch` (owns elevation, `uav_prior`) ·
  `src/mobile_manipulator/worlds/outdoor_city.world` (landmine) ·
  `src/mobile_manipulator/scripts/ugv_target_tour.py` (`ARRIVED→GRASP`) ·
  `src/mobile_manipulator/urdf/husky_ur5.urdf.xacro` (`grasp_tcp`) ·
  `src/husky_ur5_moveit_config/config/husky_ur5.srdf` (end-effector, named states, rename).
- **New:** `src/grasp_mtc/` (`srv/GetGrasps.srv`, `scripts/{grasp_task,analytic_grasp_server,gpd_grasp_server,
  perception}.py`, `launch/manipulation.launch`) · `scripts/landmine_color_detector.py`.
- **Launch, don't write:** `gazebo_grasp_plugin_ros/grasp_event_republisher` — already vendored in
  `src/gazebo-pkgs/`, never started. It is the only thing that makes `grasp_fix`'s attach observable from
  ROS, and the amendment's close gates on it.
- **Delete / gut:** `master_control.py` — port the perception out, drop the `move_base`/`map` nav half and
  the two magic scalars.
- **Do NOT touch:** mimic-joint plugins · the `gazebo_grasp_fix` plugin and its URDF block · gripper
  controller interfaces · `husky_ur5_gpd.cfg` hand geometry · the SRDF `disable_collisions` matrix · the
  landmine's friction/inertia · DLIO / `robot_body_filter` / terrain chain.
  *(`src/gazebo_link_attacher/` — the explicit-weld package built while chasing the roll — stays in-tree but
  is now **unused**: `grasp_fix` is the sole holder. Its world plugin loads and idles. Left in place because
  it is the fallback if `grasp_fix` turns out never to fire.)*

## Verification

- **Nav (Gate 1):** `rosnode list | grep elevation` → **exactly one**. FAR owns `/way_point`.
- **Reachability (Gate 2):** IK sweep for the top-down TCP pose over ground positions → reachable annulus.
- **Frames (Gate 3):** RViz TF — `grasp_tcp` between the pads.
- **Execution (Gate 4):** MTC's RViz panel shows exactly which stage failed and why.
- **Nav regression (every phase):** `/registered_scan`, `/terrain_map`, `/state_estimation` still publish;
  `view_frames` shows **no `map` frame**; DLIO still sole `odom→base_link` owner.
- **End-to-end:** `airground_takeoff.sh` → `/ugv/start_tour` → 20-trial pick campaign.

## Risks

- **`gazebo_grasp_fix` may still decline to fire.** It is the sole physical holder, and it
  has **never once fired on this rig**. The theory is that it never got the chance — we always penetrated the
  block by 16 mm and ejected it before a stable opposing contact could register — and the contact-stop ladder
  removes exactly that. If it fails, the fallback is already built and debugged
  (`src/gazebo_link_attacher/`, welding at the contact-stop point instead) — but we find out honestly, via
  `EMPTY_GRASP` / `GAZEBO_ATTACH_FAILED`, rather than papering over it.
- **Reachability** *(resolved — see Status)*: reachable band x ∈ [0.60, 1.05] m, hard cliff at 0.55, so
  `grasp_standoff = 0.75` replaced the tour's `reach_tolerance` for the grasp.
- **The duplicate `elevation_mapping` node** will silently eat one of the two maps if Phase 1.1 is skipped.
- **The landmine is one rigid link.** Grasping the detonator lifts the whole mine, with a 136 mm disc right
  under the jaws. Expect `allowCollisions` / `touch_links` pain here, not at the detonator.
- **The UGV may be parked on a slope.** `ur5_base_link` is then tilted w.r.t. gravity: the **lift must be
  along `odom` +Z, not base +Z**, or the mine gets dragged sideways into the terrain.
- **`grasp_tcp` placement is the whole ballgame.** Get it wrong and every downstream failure masquerades as a
  detector failure.
- **The CMU planner nudging the base mid-grasp** will present as a mysterious grasp-accuracy problem.
- **MTC Python bindings are v0.1.3** — thinner than C++. If `MonitoringGenerator` subclassing misbehaves,
  fall back to a plain `core.Generator` seeded with a pre-computed candidate list.
- **GraspGen checkpoint may not be public** → Phase 5.2 doesn't happen. Accepted; Phase 4 is the deliverable.

## Validated test data (teammate run `run_20260713_051123`, 2026-07-13)

Full parameter set used for the ladder (matches `_close_on_contact()` above):

```yaml
gripper_close_positions: [0.480, 0.490, 0.495, 0.498, 0.499, 0.500]
gripper_close_joint_speed: 0.06          # rad/s, normal closing speed
gripper_close_min_stage_duration: 0.60   # s, minimum time per rung
gripper_contact_settle_duration: 0.35    # s, contact-poll window per rung
gripper_contact_squeeze_delta: 0.003     # rad, post-contact squeeze
gripper_secure_joint_speed: 0.01         # rad/s, squeeze speed
gripper_secure_hold_duration: 1.0        # s, continuous-attached requirement
gripper_secure_timeout: 2.5              # s, total squeeze timeout
empty_position_tolerance: 0.025          # rad, empty-grasp threshold
grasp_event_timeout: 4.0                 # s, wait for first contact event
gripper_reset_open_attempts: 3
gripper_reset_open_tolerance: 0.030      # rad
gripper_reset_retry_delay: 0.30          # s
```

10/10 successful grasps from the best run:

| # | commanded q (1st rung) | measured contact q | q after squeeze | lift (mm) | 3 s drift (mm) |
|---:|---|---|---|---|---|
| 1 | 0.480 | 0.47948 | 0.48248 | 80.0 | 0.420 |
| 2 | 0.490 | 0.48925 | 0.49225 | 80.1 | 0.163 |
| 3 | 0.480 | 0.47982 | 0.48282 | 80.1 | 0.065 |
| 4 | 0.490 | 0.48937 | 0.49237 | 80.2 | 0.059 |
| 5 | 0.480 | 0.47943 | 0.48243 | 80.0 | 0.503 |
| 6 | 0.480 | 0.47948 | 0.48248 | 80.0 | 0.257 |
| 7 | 0.480 | 0.47920 | 0.48220 | 80.0 | 0.291 |
| 8 | 0.480 | 0.47939 | 0.48239 | 80.1 | 0.119 |
| 9 | 0.480 | 0.47972 | 0.48272 | 80.2 | 0.054 |
| 10 | 0.490 | 0.48932 | 0.49232 | 80.2 | 0.079 |

Contact landed at 0.47920–0.48937 rad across the 10 runs (~1.5 mm of gap spread) — confirming yaw genuinely
shifts the contact point and a fixed joint target cannot work. 8/10 contacted on the first rung (0.480), 2/10
on the second (0.490); none needed a third rung, so the 6-rung ladder has comfortable margin. Every run held
the 0.003 rad squeeze and stayed within 0.5 mm of drift over the 3 s hold check.

Old fixed-`q=0.513` approach vs. this ladder, on the same rig:

| | Fixed joint value (old) | Contact-adaptive ladder (this) |
|---|---|---|
| Target joint | `q = 0.513`, hardcoded | none — set by physical contact |
| Gap | fixed 36.996 mm | adaptive 39.5–43.5 mm |
| Over-close risk | 3 mm into the block, ejects the mine | 0.1 mm squeeze, never ejects |
| Empty-grasp risk | rotated detonator can miss entirely | keeps closing until contact is detected |
| Failure mode | `EMPTY_GRASP` / mine launched | `EMPTY_GRASP` only if no contact through the full ladder |
| Success rate | ~12.5% (1/8) | 100% (10/10, this run) |

(Source: `docs/UGV定点抓取雷达测试方法总结 (2).md`, a teammate write-up; folded in here and the original file
removed once every number in it was captured above.)

## Out of scope

The **real UAV vision detector** (separate work item; the `/detected_targets` seam is already specified, and
the color mock fills it). Language-conditioned grasping; suction/dexterous grippers; closed-loop visual
servoing; retraining any detector.
