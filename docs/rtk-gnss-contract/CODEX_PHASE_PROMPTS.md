# Codex Phase Prompts — RTK/GNSS Contract

Run one prompt at a time in a local checkout of `Aldoubt/agt_ins_driver` on branch `feat/rtabmap-gnss-contract`.

Before every phase read:

- `AGENTS.md`
- `RTK_GNSS_CONTRACT.md`
- this file
- the previous phase record, if present

Do not mix runtime/Nav2/HMI/camera changes into this repository.

## R0 — Repository and Protocol Truth Audit

```text
You are working in Aldoubt/agt_ins_driver.

Required branch:
  feat/rtabmap-gnss-contract

This is R0. Do not change parser semantics, ROS message definitions or published topic behavior yet. Audit the current repository/protocol truth first.

Read:
  AGENTS.md
  RTK_GNSS_CONTRACT.md
  docs/rtk-gnss-contract/CODEX_PHASE_PROMPTS.md

Tasks:
1. Confirm git branch, HEAD, remotes and clean/dirty state.
2. Build the package from a clean enough ROS 2 Humble environment:
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install --packages-select agt_asensing_driver
   source install/setup.bash
   colcon test --packages-select agt_asensing_driver
   colcon test-result --verbose
   If the repository layout requires another exact build command, record why.
3. Inspect and summarize these actual files:
   - agt_asensing_driver/src/asensing_parser.cpp
   - agt_asensing_driver/include/agt_asensing_driver/asensing_parser.hpp
   - agt_asensing_driver/include/agt_asensing_driver/ins_data.hpp
   - agt_asensing_driver/src/asensing_node.cpp
   - agt_asensing_driver/msg/INSStatus.msg
   - agt_asensing_driver/config/asensing.yaml
   - agt_asensing_driver/launch/asensing.launch.py
   - agt_asensing_driver/test/test_asensing_parser.cpp
4. Document packet header, main/extended lengths, XOR checksum coverage as implemented, all parsed byte offsets, scaling equations and selector values currently handled.
5. Explicitly trace the lifetime of INSData for successive packets. Confirm whether a new INSData is created per packet and identify which fields therefore reset between selectors.
6. Create a selector-to-fields table for selectors currently handled (including 0, 1, 2, 22, 32, 33 if still present in source).
7. Trace how position_type, rtk_fixed, satellite count, standard deviations, GPS week/time, temperature and wheel status reach /ins/status.
8. Trace how NavSatFix covariance and frame_id are currently assigned.
9. Trace /ins/pose, /ins/velocity and /ins/odom frame/coordinate semantics exactly as implemented; identify ambiguities without trying to redesign them yet.
10. Search the repository for consumers/tests/docs that depend on the current INSStatus field names and frame_id parameter.
11. Identify locally available ASENSING protocol documentation or captured serial data. If none is available, explicitly mark enum/time semantics that cannot be proven from source alone.
12. Do not use internet assumptions to rewrite the protocol in this phase.
13. Create docs/rtk-gnss-contract/records/R00_TRUTH_AUDIT.md containing:
    - git/build/test state;
    - packet/parser table;
    - selector/state-lifetime analysis;
    - current ROS topic/message/frame table;
    - current covariance mapping;
    - current timestamp behavior;
    - identified correctness defects;
    - protocol facts that are verified vs still uncertain;
    - exact R1 and R2 file-level recommendations.
14. Update only the Current Status table in RTK_GNSS_CONTRACT.md if R0 passes.
15. Show the audit summary and diff before commit.
16. Commit only documentation/audit changes:
    docs: audit asensing gnss contract baseline

Acceptance:
- No runtime behavior changed.
- Every planned R1/R2 change is tied to a documented source defect or explicit ROS-contract requirement.
- Unknown vendor semantics are clearly marked unknown instead of guessed.
```

## R1 — Persistent Parser/Auxiliary State Correctness

```text
You are implementing R1 in Aldoubt/agt_ins_driver on branch feat/rtabmap-gnss-contract.

Read AGENTS.md, RTK_GNSS_CONTRACT.md and docs/rtk-gnss-contract/records/R00_TRUTH_AUDIT.md.

Goal:
Fix the current selector-driven metadata reset problem so emitted INS state retains the latest valid auxiliary values across packets.

Hard constraints:
- Preserve verified packet framing, byte offsets, scaling and XOR behavior from R0.
- Do not change NavSatFix covariance/frame semantics yet unless required only to make tests compile; those are R2.
- Do not invent vendor enum meanings.
- Do not add a large health state machine.
- Do not move RTAB/Nav2 logic into this driver.

Preferred design:
Keep protocol parsing and latest-state aggregation deterministic and unit-testable. A parser-owned latest auxiliary state or a small dedicated aggregation object is acceptable. The design must make it obvious which selector updates which fields.

TDD requirements:
1. Extend agt_asensing_driver/test/test_asensing_parser.cpp first with packet-sequence tests that fail on the current implementation.
2. Reuse or add deterministic packet-builder helpers in the test file so selector packets can be generated with valid checksums and known values.
3. Add a sequence test:
   packet(selector 0 with position std A)
   packet(selector 32 with position type/satellite/heading B)
   packet(selector 1 with velocity std C)
   packet(selector 2 with attitude std D)
   Verify the final emitted state still contains A+B+C+D.
4. Add a test that a second selector 32 packet updates only solution-status fields while preserving A/C/D.
5. Add a test that a second selector 0 packet updates only position std while preserving solution/velocity/attitude metadata.
6. Add a test that a selector 22/33 packet does not erase position/solution/std metadata.
7. Add a test for GPS week/time behavior so the state-persistence refactor does not regress current packet framing/extension parsing.
8. If validity flags are introduced, test initial not-yet-seen state and seen/update behavior explicitly.
9. Run the tests and confirm the new tests fail before implementation.
10. Implement the smallest parser/state change to pass the tests.
11. Run:
    colcon build --symlink-install --packages-select agt_asensing_driver
    colcon test --packages-select agt_asensing_driver
    colcon test-result --verbose
12. Inspect the implementation for accidental sharing of stale main-packet fields. Main measurement values such as roll/pitch/yaw/lat/lon/velocity must still update per packet; persistence is for asynchronous auxiliary metadata, not for suppressing new main measurements.
13. Create docs/rtk-gnss-contract/records/R01_PARSER_STATE.md containing:
    - old failure mechanism;
    - new state model;
    - selector update table;
    - test cases and results;
    - any validity/freshness semantics added;
    - known protocol uncertainties left unchanged.
14. Update RTK_GNSS_CONTRACT.md R1 status only after all tests pass.
15. Show diff/test evidence before commit.
16. Commit:
    fix(parser): persist asensing auxiliary metadata

Acceptance:
- Auxiliary metadata no longer returns to default zero solely because a different selector arrived.
- Unit tests prove cross-selector persistence and selective updates.
- Verified protocol parsing behavior is not otherwise changed.
```

## R2 — ROS GNSS Interface Correctness

```text
You are implementing R2 in Aldoubt/agt_ins_driver on branch feat/rtabmap-gnss-contract.

Read AGENTS.md, RTK_GNSS_CONTRACT.md, R00 and R01 records.

Goal:
Make /ins/navsatfix and /ins/status trustworthy ROS 2 contracts while keeping /ins/pose, /ins/velocity and /ins/odom explicitly limited/compatibility outputs.

Hard constraints:
- Do not implement a new local/global fusion architecture.
- Do not convert WGS84 coordinates to local map/odom coordinates in this phase.
- Do not claim generic NavSatFix STATUS_FIX means RTK fixed.
- Do not convert GPS week/time to ROS absolute time without verified device-time semantics.
- Do not hard-code a robot-specific lever arm.
- Do not invent ENU/NED heading conventions.

Required changes:
1. Replace/augment the single frame_id parameter with explicit parameters:
   ins_frame_id: ins_link
   gnss_frame_id: rtk_antenna_link
   Preserve a documented compatibility path only if necessary for existing launch/config users.
2. Publish NavSatFix.header.frame_id from gnss_frame_id.
3. Correct NavSatFix covariance order to East, North, Up. Based on the existing field names, map longitude_std to East variance, latitude_std to North variance and altitude_std to Up variance, unless R0 vendor evidence proves different semantics.
4. Set covariance type according to actual validity. Do not advertise known diagonal covariance before the corresponding std group has been received.
5. Add minimal finite/range sanity handling for geodetic values. Do not silently publish obviously invalid values as a good fix.
6. Keep generic NavSatStatus and normalized RTK-fixed status conceptually separate.
7. Make /ins/status stable using the persisted R1 state.
8. Improve INSStatus.msg so position quality is explicit. Prefer named east/north/up standard deviation fields rather than one ambiguous position_std. Keep raw solution/status fields required for diagnostics.
9. Update asensing_node.cpp, message definitions, config, launch and README together.
10. Add/update automated tests. If direct node-level ROS tests are excessive for this small package, factor pure helper functions for covariance/status mapping and unit-test those, or add a small gtest around message-construction logic. Do not leave semantics untested merely because the current node is monolithic.
11. Explicitly document /ins/velocity semantics as navigation-frame north/east/ground quantities unless R0 evidence says otherwise. Do not call them body-frame x/y/z in documentation.
12. Explicitly document /ins/odom as compatibility/experimental because it has no accepted local-position origin. Do not present it as the runtime's odom -> base source.
13. Keep timestamp default as ROS receive time. Preserve gps_week/gps_time_ms. If adding timestamp_source, support ros_receive as default and do not implement gps_time conversion until verified.
14. Add configuration fields for lever-arm capability only if the design can keep compensation disabled by default and robot-specific values external. It is acceptable in R2 to limit the change to correct gnss_frame_id semantics and defer mathematical compensation.
15. Build/test:
    colcon build --symlink-install --packages-select agt_asensing_driver
    colcon test --packages-select agt_asensing_driver
    colcon test-result --verbose
16. If recorded serial/ROS data is locally available, replay/run against it and inspect /ins/navsatfix and /ins/status for stable covariance/status behavior. Record exact data source.
17. Create docs/rtk-gnss-contract/records/R02_ROS_CONTRACT.md with:
    - final public topic table;
    - frame semantics;
    - covariance equation/order;
    - status field definitions;
    - timestamp policy;
    - /pose /velocity /odom limitations;
    - test/build/replay evidence;
    - migration notes for any INSStatus field change.
18. Update RTK_GNSS_CONTRACT.md R2 status only when accepted.
19. Show diff/evidence before commit.
20. Use coherent commits; if message-schema and node changes are inseparable, one commit is acceptable:
    fix(gnss): correct navsatfix frame and covariance semantics
    or follow with:
    feat(status): expose stable gnss quality metadata

Acceptance:
- Downstream runtime can consume /ins/navsatfix and /ins/status without depending on selector timing.
- NavSatFix uses GNSS antenna frame and correct ENU covariance semantics.
- RTK fixed remains separately observable from generic NavSat status.
- No unverified GPS absolute-time conversion was introduced.
```

## R3 — Real RTK/INS Hardware Validation and Freeze Decision

```text
You are performing R3 in Aldoubt/agt_ins_driver on branch feat/rtabmap-gnss-contract.

Read AGENTS.md, RTK_GNSS_CONTRACT.md and R00-R02 records.

Goal:
Validate the R2 contract against the real ASENSING receiver/RTK setup and decide whether the branch is ready for a candidate stable SHA/tag.

This phase is evidence-first. Do not redesign the driver while collecting baseline data unless a concrete defect appears.

Preparation:
1. Record current branch, HEAD, build command, config file and physical serial device/baudrate.
2. Record the vehicle/sensor setup including where the GNSS antenna is mounted relative to base_link. If exact lever-arm dimensions are not yet measured, measure/document x/y/z in the robot frame before calling the frame geometry complete.
3. Record whether the test has RTK correction service/base available and what receiver solution modes are expected.

Tests:
A. Static open-sky or best available static test
- run long enough to observe solution stabilization;
- record /ins/navsatfix rate and /ins/status rate;
- record position_type transitions;
- record rtk_fixed transitions;
- record satellite count;
- record east/north/up std values;
- record latitude/longitude/altitude scatter;
- verify status/std fields do not periodically drop to zero due to selector rotation.

B. Short controlled motion test
- move the platform along a simple path with known qualitative direction;
- verify latitude/longitude change direction is plausible;
- inspect orientation/velocity only as diagnostics; do not promote /ins/odom to a navigation contract;
- check message timestamp monotonicity and receive-time behavior.

C. RTK degradation/recovery if practical
- observe float/fixed transition or temporarily remove corrections/antenna visibility safely;
- verify raw position_type and normalized rtk_fixed change coherently and recover;
- verify stale fixed status is not retained forever when the receiver actually reports a new non-fixed solution.

D. Serial reconnect
- disconnect/reconnect or restart the node safely;
- verify the driver resumes publication without corrupt state;
- document whether auxiliary metadata is considered invalid until each group is seen after reconnect.

E. Device-time observation
- log gps_week, gps_time_ms and ROS header stamps together;
- calculate/plot the observed relationship over the test;
- do not switch to GPS-time stamping unless the epoch/UTC offset behavior is proven.

Artifacts:
- save a rosbag2 containing at least /ins/navsatfix, /ins/status and relevant /tf_static for the hardware validation where storage permits;
- do not commit large bag data into git; record its path/name/hash/metadata in the report.

Deliverable:
Create docs/rtk-gnss-contract/records/R03_HARDWARE_VALIDATION.md containing:
- hardware setup;
- branch/HEAD/config;
- test durations;
- rate/statistics tables;
- RTK fixed/float/degraded observations;
- covariance/std behavior;
- timestamp observations;
- reconnect behavior;
- antenna frame/lever-arm measurement;
- defects found and fixes/commits if any;
- validation label;
- freeze recommendation: NOT_READY, CANDIDATE_SHA, or READY_FOR_TAG.

If defects require code changes:
- make the smallest focused fix;
- add a regression test when possible;
- rerun the relevant R3 subset;
- update the report with before/after evidence.

Only after evidence supports it, update RTK_GNSS_CONTRACT.md R3 and freeze status.

Suggested validation commit when no code change is needed:
  test(gnss): validate hardware fix and timestamp behavior

Do not create a stable release/tag merely because the build passes.
```

## End-of-Phase Review Prompt

```text
Review the completed RTK/GNSS contract phase as a skeptical ROS navigation and sensor-driver maintainer.

Read AGENTS.md, RTK_GNSS_CONTRACT.md, the phase record, current diff/commit and tests.

Report blockers first. Check for:
- guessed vendor protocol/enum/time semantics;
- auxiliary fields still resetting across selectors;
- longitude/latitude std mapped to the wrong ENU covariance slots;
- NavSatFix using a generic INS frame instead of antenna frame;
- generic STATUS_FIX being documented as RTK fixed;
- an ambiguous position_std field still representing altitude only;
- /ins/velocity being mislabeled as body-frame velocity;
- /ins/odom being presented as accepted local odometry without an origin;
- unverified GPS-to-Unix time conversion;
- robot-specific lever arm hard-coded in the driver;
- STATIC/BAG evidence mislabeled VEHICLE_VALIDATED;
- premature SHA/tag freeze.

Do not begin the next phase. Make only phase-local corrections, rerun tests/evidence, and update the record.
```
