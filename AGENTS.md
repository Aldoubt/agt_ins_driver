# AGT INS/GNSS Driver Agent Guide

This file defines hard constraints for Codex and other agentic developers working on the GNSS/INS correctness workstream in `agt_ins_driver`.

## Active Workstream

- Repository: `Aldoubt/agt_ins_driver`
- Branch: `feat/rtabmap-gnss-contract`
- Base at branch creation: `master@8d8ebe73ddebd6f68b6acafd02dd9f559a67d89c`
- Primary design/status document: `RTK_GNSS_CONTRACT.md`
- Phase prompts: `docs/rtk-gnss-contract/CODEX_PHASE_PROMPTS.md`
- Implementation plan: `docs/superpowers/plans/2026-09-02-rtk-gnss-contract.md`
- SHA state: UNFROZEN until acceptance passes

## Mission

Turn the ASENSING driver into a trustworthy, self-contained ROS 2 GNSS/INS capability before it is treated as a global prior by navigation systems.

This repository owns protocol parsing and ROS semantic correctness. It must not contain RTAB-Map, Nav2, HMI, mission, camera, or robot-specific navigation logic.

## Public Contract for the First Runtime Integration

The first external consumers are allowed to rely on:

- `/ins/navsatfix` (`sensor_msgs/msg/NavSatFix`)
- `/ins/status` (`agt_asensing_driver/msg/INSStatus`)

The following existing topics remain compatibility/experimental outputs until their coordinate semantics are explicitly accepted:

- `/ins/pose`
- `/ins/velocity`
- `/ins/odom`

Do not make those three topics part of a new navigation contract merely because they already exist.

## Protocol Fidelity Rule

Do not guess ASENSING packet offsets, checksum coverage, units, enum meanings, GPS-time epoch, or solution-type semantics.

When changing protocol behavior:

1. identify the exact existing source behavior;
2. preserve verified packet framing/checksum logic;
3. use vendor documentation or captured packet evidence for any changed interpretation;
4. add a parser test that reproduces the packet/block sequence;
5. document any still-unknown enum/time meaning instead of inventing it.

No ROS integration goal justifies corrupting the protocol layer.

## Persistent Auxiliary State Rule

The current protocol exposes different metadata groups in different extended/selector blocks. A consumer-facing state must not reset unrelated metadata to zero merely because the current packet carries another selector.

The implementation must maintain latest-valid auxiliary values across packets, with explicit validity/freshness tracking where needed.

At minimum cover:

- position standard deviations;
- velocity standard deviations;
- attitude standard deviations;
- position/solution type;
- satellite count;
- heading type;
- temperature;
- wheel-speed status;
- GPS week/time metadata.

Do not hide persistence bugs in the ROS node by hard-coding defaults. Prefer a clear state aggregation design that is unit-testable.

## NavSatFix Semantics

`/ins/navsatfix` must follow ROS `sensor_msgs/NavSatFix` semantics.

Required rules:

- latitude/longitude/altitude remain WGS84/geodetic outputs from the receiver;
- `header.frame_id` denotes the GNSS antenna measurement location, not a generic INS frame;
- position covariance axes are East, North, Up in row-major order;
- if latitude std represents North and longitude std represents East, map them accordingly;
- covariance type must match the data actually known;
- invalid/non-finite coordinates must not be presented as a trustworthy usable fix;
- solution/status meaning must be explicit and tested.

Do not claim `STATUS_FIX` means RTK fixed. RTK fixed belongs in the richer status contract.

## Frame Semantics

Separate at least the concepts of:

- `ins_frame_id`: the INS/body sensor frame;
- `gnss_frame_id`: the GNSS antenna frame, expected to be `rtk_antenna_link` on the target robot.

Do not reuse one `frame_id` for every message when the physical/reference semantics differ.

This repository may expose frame parameters. Robot-specific measured transforms belong to robot description/profile configuration in the runtime/vehicle repository, not hard-coded here.

## Velocity Semantics

The parser exposes north/east/ground velocity quantities. Do not label these as body-frame x/y/z velocities without proof.

Before changing `/ins/velocity` or `/ins/odom` semantics, document:

- source coordinate convention (ENU/NED/vendor navigation frame);
- axis mapping;
- vertical/ground-speed meaning;
- header frame meaning.

For the first RTAB integration, runtime will not consume these topics, so do not over-expand scope.

## Pose/Odometry Semantics

An orientation-only `nav_msgs/Odometry` message is not automatically a valid `odom -> child` local odometry source.

Do not add invented local positions derived from latitude/longitude without an explicit origin/conversion design.

If `/ins/odom` remains compatibility output, document its limitations instead of pretending it is a fully defined local odometry frame.

## Timestamp Policy

The current driver stamps ROS messages at receive/publish time while preserving GPS week/time metadata.

For the first accepted contract:

- keep `ros_receive` as the default timestamp source unless GPS-time conversion is verified;
- preserve raw GPS week/time metadata;
- do not convert GPS week/time to Unix/ROS time without verifying epoch, rollover, UTC/GPS offset and leap-second handling;
- if a selectable timestamp source is added, make it explicit and testable;
- record receive-vs-device-time observations during hardware validation.

A wrong absolute timestamp is worse than a documented receive timestamp.

## Lever Arm Policy

This repository may expose configuration/capability for GNSS antenna lever arm, but compensation is OFF by default in the first workstream.

Required concepts:

- GNSS antenna frame is explicit;
- measured `base_link -> rtk_antenna_link` is owned by vehicle geometry, not hard-coded in this driver;
- any antenna-to-reference correction requires a verified attitude/navigation convention;
- do not implement ENU/NED/yaw compensation by assumption.

Static TF documentation and mathematical compensation are separate issues.

## INSStatus Contract

The status message must expose enough information to judge GNSS quality without forcing consumers to reverse-engineer vendor packet selectors.

Prefer explicit fields over ambiguous aggregation. In particular, do not use one field called `position_std` to represent only altitude std while consumers may interpret it as horizontal position quality.

Any message change must:

- be documented;
- update publishers;
- update tests;
- preserve or deliberately migrate consumers;
- avoid needless robot-specific policy.

## RTK Fixed Policy

`rtk_fixed_types` is a configurable mapping from vendor solution type(s) to the normalized `rtk_fixed` flag.

Do not assume the enum list is universally correct without vendor/captured-data evidence. Keep the raw `position_type` visible even when publishing the normalized boolean.

The normalized boolean must remain stable between selector packets by using persisted solution metadata.

## Health Policy

Keep the first contract minimal. Do not add a large GOOD/DEGRADED/BAD state machine.

The driver should make raw/normalized evidence available and reject obviously invalid values. Higher-level health policy can be built later by consumers.

## Testing Rules

Parser and state aggregation changes require deterministic unit tests.

At minimum add tests for sequences where selector blocks alternate, proving that previously received metadata persists correctly while newly received blocks update only their own fields.

ROS semantic tests should cover:

- ENU covariance mapping;
- distinct INS/GNSS frame ids;
- RTK fixed persistence;
- non-finite/invalid fix handling;
- status field mapping;
- timestamp-source default behavior if configurable.

Hardware validation is separate from unit tests.

## Validation Labels

Use only:

- `STATIC_VALIDATED`: compile/unit/static-contract checks passed;
- `BAG_VALIDATED`: recorded serial/ROS data replay passed;
- `VEHICLE_VALIDATED`: real receiver/vehicle test passed.

Do not call parser unit tests a vehicle validation.

## Phase Order

Follow `RTK_GNSS_CONTRACT.md`:

1. R0 truth audit;
2. R1 persistent parser/state correctness;
3. R2 ROS GNSS contract correctness;
4. R3 hardware/real-data validation and freeze decision.

Do not pin a final SHA before R3 or an explicitly accepted equivalent hardware evidence gate.

## Codex Workflow

Before modifying a phase:

1. read this file and `RTK_GNSS_CONTRACT.md`;
2. confirm branch and git state;
3. inspect current parser/node/message/config/tests;
4. state the exact protocol/ROS semantic defect being addressed;
5. write or extend a failing test first where practical;
6. make the smallest implementation change;
7. run package build and tests;
8. record evidence in `docs/rtk-gnss-contract/records/`;
9. update the status table;
10. commit the coherent phase.

Do not mix navigation-runtime changes into this repository.

## Stop Conditions

Stop and investigate instead of guessing when:

- vendor enum meaning is uncertain;
- checksum/packet-length behavior is ambiguous;
- a selector block interpretation is unsupported by tests/docs;
- GPS-time conversion assumptions are unverified;
- frame convention is ambiguous;
- RTK fixed status flickers because metadata was not persisted;
- covariance becomes zero solely because another selector packet arrived;
- real hardware values contradict unit-test assumptions.

## Commit Policy

Keep commits phase-focused. Suggested progression:

```text
docs: audit asensing gnss contract baseline
fix(parser): persist asensing auxiliary metadata
fix(gnss): correct navsatfix frame and covariance semantics
feat(status): expose stable gnss quality metadata
test(gnss): validate hardware timestamp and fix behavior
```

Do not include unrelated runtime/HMI/camera changes.
