# RTK/GNSS Contract Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `agt_ins_driver` publish a stable, ROS-correct `/ins/navsatfix` and `/ins/status` contract whose metadata does not flicker with ASENSING selector packets.

**Architecture:** Preserve verified ASENSING protocol framing/scaling, add deterministic persistence for asynchronous auxiliary metadata, then correct ROS frame/covariance/status semantics in the node layer. Keep receive-time stamping as the safe default until device-time conversion is verified, and keep robot-specific lever-arm values outside this repository.

**Tech Stack:** ROS 2 Humble, C++17, rclcpp, sensor_msgs/NavSatFix, geometry_msgs, nav_msgs, custom `INSStatus`, gtest, serial ASENSING protocol parser.

**Spec:** `RTK_GNSS_CONTRACT.md`

## Global Constraints

- Work only on `feat/rtabmap-gnss-contract` for this plan.
- Do not add RTAB-Map, Nav2, HMI, camera or mission logic.
- Do not guess vendor packet offsets, enum meanings or GPS-time semantics.
- `/ins/navsatfix` and `/ins/status` are the first accepted downstream contracts.
- `/ins/pose`, `/ins/velocity` and `/ins/odom` remain compatibility/experimental until separately accepted.
- `NavSatFix.position_covariance` is ENU.
- GNSS antenna frame and INS frame are distinct concepts.
- Do not freeze a final SHA/tag before R3 hardware evidence.

---

### Task 1: R0 Repository and Protocol Truth Audit

**Files:**
- Create: `docs/rtk-gnss-contract/records/R00_TRUTH_AUDIT.md`
- Modify: `RTK_GNSS_CONTRACT.md`

**Interfaces:**
- Consumes: current parser/node/config/message/tests and any local vendor/captured evidence.
- Produces: verified packet/selector/state-lifetime table and exact R1/R2 change list.

- [ ] **Step 1: Verify git/build state**

```bash
git status --short --branch
git rev-parse HEAD
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agt_asensing_driver
colcon test --packages-select agt_asensing_driver
colcon test-result --verbose
```

- [ ] **Step 2: Map parser bytes and selectors**

Read `agt_asensing_driver/src/asensing_parser.cpp` and record header bytes, main/extended lengths, checksum coverage, every parsed offset/scale and every selector branch.

- [ ] **Step 3: Prove the reset mechanism**

Trace `INSData d;` lifetime for two consecutive packets with different selectors and list fields that return to default because they are not filled in the second packet.

- [ ] **Step 4: Audit node semantics**

Trace frame ids, NavSatFix covariance assignment, generic fix status, RTK fixed mapping, velocity/odom meaning, `INSStatus.position_std`, and timestamp assignment in `asensing_node.cpp`.

- [ ] **Step 5: Audit compatibility surface**

Search all repo files for:

```bash
grep -R "position_std\|frame_id\|/ins/odom\|/ins/velocity\|rtk_fixed" -n . --exclude-dir=build --exclude-dir=install --exclude-dir=log
```

Record message/config changes that would require migration.

- [ ] **Step 6: Write R0 record and update status**

`R00_TRUTH_AUDIT.md` must distinguish verified protocol facts from unknown vendor semantics.

- [ ] **Step 7: Commit**

```bash
git add RTK_GNSS_CONTRACT.md docs/rtk-gnss-contract/records/R00_TRUTH_AUDIT.md
git commit -m "docs: audit asensing gnss contract baseline"
```

---

### Task 2: R1 Persist Auxiliary Metadata Across Selector Packets

**Files:**
- Modify: `agt_asensing_driver/include/agt_asensing_driver/ins_data.hpp`
- Modify: `agt_asensing_driver/include/agt_asensing_driver/asensing_parser.hpp`
- Modify: `agt_asensing_driver/src/asensing_parser.cpp`
- Modify: `agt_asensing_driver/test/test_asensing_parser.cpp`
- Create: `docs/rtk-gnss-contract/records/R01_PARSER_STATE.md`
- Modify: `RTK_GNSS_CONTRACT.md`

**Interfaces:**
- Consumes: ASENSING byte stream.
- Produces: emitted `INSData` whose main measurements update each packet while asynchronous auxiliary metadata persists until replaced.

- [ ] **Step 1: Add a failing alternating-selector test**

Add a deterministic packet-builder helper to `test_asensing_parser.cpp` if one does not already exist. Feed selector sequence `0 -> 32 -> 1 -> 2` with distinct known values and assert the final emitted state contains all four groups.

Expected before implementation: FAIL because unrelated groups reset to defaults.

- [ ] **Step 2: Add selective-update tests**

Test:

```text
0(A) -> 32(B) -> 0(A2)
```

Expected final state: position std=A2, solution=B.

Test selector 22/33 packets do not erase position/std/solution groups.

- [ ] **Step 3: Add extension/time regression test**

Verify existing GPS week/time extension parsing and packet consumption still behave identically after the persistence design.

- [ ] **Step 4: Run tests and verify failure**

```bash
colcon test --packages-select agt_asensing_driver
colcon test-result --verbose
```

- [ ] **Step 5: Implement the smallest state model**

Use parser-owned latest auxiliary state or a small explicit aggregation state. Each selector updates only its fields. Main roll/pitch/yaw/gyro/accel/lat/lon/alt/velocity values continue to come from the current packet.

If validity flags are added, make them per logical group rather than one ambiguous all-valid boolean.

- [ ] **Step 6: Rebuild and run tests**

```bash
colcon build --symlink-install --packages-select agt_asensing_driver
colcon test --packages-select agt_asensing_driver
colcon test-result --verbose
```

Expected: all previous and new parser tests PASS.

- [ ] **Step 7: Write R1 evidence and commit**

```bash
git add agt_asensing_driver/include agt_asensing_driver/src/asensing_parser.cpp \
  agt_asensing_driver/test/test_asensing_parser.cpp RTK_GNSS_CONTRACT.md \
  docs/rtk-gnss-contract/records/R01_PARSER_STATE.md
git commit -m "fix(parser): persist asensing auxiliary metadata"
```

---

### Task 3: R2 Correct ROS GNSS/Status Contract

**Files:**
- Modify: `agt_asensing_driver/src/asensing_node.cpp`
- Modify: `agt_asensing_driver/msg/INSStatus.msg`
- Modify: `agt_asensing_driver/config/asensing.yaml`
- Modify: `agt_asensing_driver/launch/asensing.launch.py`
- Modify: `agt_asensing_driver/README.md` if present, otherwise repository `README.md`
- Modify: `README.md`
- Add focused tests under: `agt_asensing_driver/test/`
- Modify: `agt_asensing_driver/CMakeLists.txt`
- Create: `docs/rtk-gnss-contract/records/R02_ROS_CONTRACT.md`
- Modify: `RTK_GNSS_CONTRACT.md`

**Interfaces:**
- Consumes: stable R1 `INSData`.
- Produces: accepted `/ins/navsatfix` and `/ins/status` contracts.

- [ ] **Step 1: Factor testable ROS-semantic helpers if needed**

If `asensing_node.cpp` is too monolithic for unit tests, introduce a small header/source helper such as:

```text
agt_asensing_driver/include/agt_asensing_driver/message_mapping.hpp
agt_asensing_driver/src/message_mapping.cpp
```

with pure functions that map `INSData` + configuration to covariance/status fields. Keep serial I/O out of this helper.

- [ ] **Step 2: Write failing covariance tests**

Given:

```text
latitude_std = 0.20  # North
longitude_std = 0.10 # East
altitude_std = 0.30  # Up
```

assert NavSatFix covariance diagonal is:

```text
[0] = 0.01
[4] = 0.04
[8] = 0.09
```

and only advertise known diagonal covariance after the position-std group is valid.

- [ ] **Step 3: Write failing frame/status tests**

Assert:

```text
NavSatFix.header.frame_id == gnss_frame_id
pose/orientation compatibility output uses ins_frame_id where semantically appropriate
rtk_fixed remains stable from R1 position_type
```

- [ ] **Step 4: Replace single frame parameter with explicit frame concepts**

Default config:

```yaml
ins_frame_id: ins_link
gnss_frame_id: rtk_antenna_link
```

If a temporary legacy `frame_id` fallback is necessary, document precedence and deprecation; do not keep ambiguity silently.

- [ ] **Step 5: Improve `INSStatus.msg`**

Replace or augment ambiguous position quality with explicit fields. Preferred names:

```text
float32 east_std
float32 north_std
float32 up_std
```

Retain raw `position_type`, `heading_type`, `num_satellite`, `rtk_fixed`, GPS week/time, temperature and wheel-speed status. If keeping `position_std` for compatibility, document its deprecation and do not fill it with an undocumented altitude-only meaning.

- [ ] **Step 6: Add minimal fix sanity handling**

Reject or mark unusable non-finite/out-of-range geodetic values. Keep NavSat generic fix state conceptually separate from `rtk_fixed`.

- [ ] **Step 7: Preserve safe timestamp policy**

Continue ROS receive-time stamping by default and retain GPS week/time metadata. Do not add GPS absolute-time conversion without R0/R3 evidence.

- [ ] **Step 8: Document compatibility topics**

State clearly that `/ins/velocity` contains navigation-frame north/east/ground quantities as currently parsed and that `/ins/odom` lacks an accepted local-position origin; they are not the first runtime navigation contract.

- [ ] **Step 9: Build/test**

```bash
colcon build --symlink-install --packages-select agt_asensing_driver
colcon test --packages-select agt_asensing_driver
colcon test-result --verbose
```

- [ ] **Step 10: Replay recorded data if available**

Verify `/ins/navsatfix` and `/ins/status` do not periodically lose covariance/status because selector blocks rotate.

- [ ] **Step 11: Write R2 record and commit**

Use one or two coherent commits depending on message-schema scope:

```bash
git commit -m "fix(gnss): correct navsatfix frame and covariance semantics"
```

and, if separated:

```bash
git commit -m "feat(status): expose stable gnss quality metadata"
```

---

### Task 4: R3 Hardware Validation and Freeze Decision

**Files:**
- Create: `docs/rtk-gnss-contract/records/R03_HARDWARE_VALIDATION.md`
- Modify: `RTK_GNSS_CONTRACT.md`
- Modify source/tests only if a concrete R3 defect is found.

**Interfaces:**
- Consumes: real ASENSING serial data and target vehicle GNSS antenna geometry.
- Produces: hardware evidence and a freeze recommendation, not necessarily a code change.

- [ ] **Step 1: Record hardware/config identity**

Record receiver model/setup, serial port, baudrate, correction availability, branch HEAD and exact config file.

- [ ] **Step 2: Measure/document antenna geometry**

Record the physical `base_link -> rtk_antenna_link` x/y/z used by the target vehicle. Keep this measurement in the R3 report; the robot-specific transform itself belongs in the vehicle/runtime description repository.

- [ ] **Step 3: Static acquisition test**

Record topic rates, solution transitions, satellite count, East/North/Up std and static scatter. Verify persisted metadata no longer drops to zero due solely to selector rotation.

- [ ] **Step 4: Controlled motion test**

Record qualitative path consistency and timestamp monotonicity. Inspect velocity/orientation only diagnostically; do not promote `/ins/odom` to the runtime odometry contract.

- [ ] **Step 5: Degrade/recover corrections if safe**

Observe fixed/float/non-fixed transitions and verify raw `position_type` and normalized `rtk_fixed` change coherently.

- [ ] **Step 6: Serial reconnect test**

Restart/disconnect/reconnect safely and verify publication resumes. Check whether validity state is appropriately reinitialized and document behavior.

- [ ] **Step 7: Log device vs ROS time**

Record `gps_week`, `gps_time_ms` and ROS header stamps together. Compute the observed relationship but do not switch timestamp source unless semantics are fully proven.

- [ ] **Step 8: Save a hardware-validation bag outside git**

Record bag name/path/hash/`ros2 bag info` metadata in R3. Do not commit large bags.

- [ ] **Step 9: Fix only concrete defects**

For every code fix discovered in R3, add a regression test and rerun the affected hardware check.

- [ ] **Step 10: Decide freeze state**

Write one result in the report:

```text
NOT_READY
CANDIDATE_SHA
READY_FOR_TAG
```

Only `CANDIDATE_SHA`/`READY_FOR_TAG` after real evidence supports the public contract.

- [ ] **Step 11: Commit evidence**

```bash
git add RTK_GNSS_CONTRACT.md docs/rtk-gnss-contract/records/R03_HARDWARE_VALIDATION.md
git commit -m "test(gnss): validate hardware fix and timestamp behavior"
```

---

## Final Review Before Freeze

Run:

```bash
git status --short --branch
colcon build --symlink-install --packages-select agt_asensing_driver
colcon test --packages-select agt_asensing_driver
colcon test-result --verbose
```

Review all R0-R3 records and confirm:

- selector-driven metadata persistence is tested;
- NavSatFix covariance is ENU;
- GNSS antenna frame is explicit;
- RTK fixed is distinct from generic NavSat fix;
- position standard deviations are unambiguous;
- receive-time vs GPS-time policy is documented;
- robot-specific lever arm is not hard-coded in this driver;
- `/ins/odom` is not falsely advertised as accepted local odometry;
- real hardware evidence supports any proposed stable SHA/tag.
