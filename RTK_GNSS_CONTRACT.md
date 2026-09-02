# RTK / GNSS Contract Workstream

This is the visible entry point for correcting and validating `agt_ins_driver` before its data is treated as a navigation/global-position prior.

## Current Branch

- Repository: `Aldoubt/agt_ins_driver`
- Branch: `feat/rtabmap-gnss-contract`
- Base at branch creation: `master@8d8ebe73ddebd6f68b6acafd02dd9f559a67d89c`
- Status: DESIGN FROZEN, IMPLEMENTATION NOT STARTED
- Final SHA: NOT FROZEN

The branch may evolve through R0-R3. Do not lock a consumer to a final SHA until the acceptance gates below pass.

## Purpose

The goal is not to make this repository RTAB-specific. The goal is to make it a reliable ROS 2 GNSS/INS capability that any runtime can consume.

The first downstream integration only needs a trustworthy:

```text
/ins/navsatfix
/ins/status
```

Other outputs remain available but are not accepted as navigation contracts until their semantics are reviewed.

## Current Repository Truth at Branch Creation

Current package structure:

```text
agt_asensing_driver/
  config/asensing.yaml
  include/agt_asensing_driver/asensing_parser.hpp
  include/agt_asensing_driver/ins_data.hpp
  launch/asensing.launch.py
  msg/INSStatus.msg
  src/asensing_node.cpp
  src/asensing_parser.cpp
  src/serial_port.cpp
  test/test_asensing_parser.cpp
```

The current parser creates a fresh `INSData` for every main packet and only fills the auxiliary metadata selected by the current packet's selector byte. Therefore metadata from other selector blocks is reset to default values in the next emitted sample unless it happens to be present again.

This can make standard deviations, `position_type`, `num_sv`, `heading_type`, temperature and wheel-speed metadata appear to flicker or become zero between selector packets.

The current node also:

- uses one `frame_id` parameter for GNSS fix, pose, velocity and odometry;
- publishes NavSatFix covariance with latitude variance in the first diagonal element and longitude variance in the second, while ROS NavSatFix covariance is ENU;
- maps any nonzero `ins_status` to generic `STATUS_FIX`, which does not distinguish RTK fixed;
- publishes north/east/ground velocities under the same generic frame id without an explicit body/navigation-frame contract;
- publishes `/ins/odom` without a defined local-position origin;
- sets `INSStatus.position_std` from altitude std only;
- computes `rtk_fixed` from `position_type`, but that source value currently exists only on the corresponding selector packet unless persisted;
- stamps messages with ROS `now()` while retaining GPS week/time fields.

These are repository-level correctness concerns and must be repaired here rather than patched inside navigation runtime.

## Boundary With Navigation Runtime

`Aldoubt/agt_navigation_runtime` must not copy or patch this parser.

The first accepted runtime contract is:

```text
/ins/navsatfix
  sensor_msgs/msg/NavSatFix

/ins/status
  agt_asensing_driver/msg/INSStatus
```

Runtime may inspect RTK quality from `/ins/status`, but it should not reverse-engineer ASENSING selector packets or vendor enums.

Until separately accepted, runtime must not use:

```text
/ins/pose
/ins/velocity
/ins/odom
```

as its primary navigation/odometry source.

## Required Design Corrections

### 1. Persistent auxiliary metadata

The parser/state layer must preserve the latest valid value from each auxiliary block across subsequent packets.

A selector update changes only the fields belonging to that selector. It must not implicitly zero unrelated metadata.

The design must be deterministic and unit-testable with packet sequences such as:

```text
selector 0  -> position std
selector 32 -> solution status
selector 1  -> velocity std
selector 2  -> attitude std
selector 32 -> solution status update
```

At every emitted state, previously received values remain available until replaced or explicitly invalidated by a documented policy.

### 2. Explicit validity/freshness

Do not silently confuse "not received yet" with numeric zero.

At minimum the implementation should be able to distinguish whether key auxiliary groups have ever been received. If introducing per-group timestamps/ages is useful, keep it small and testable; do not build a large health state machine in this phase.

### 3. NavSatFix covariance

ROS covariance order is East, North, Up.

If the vendor fields are interpreted as:

```text
longitude_std -> East
latitude_std  -> North
altitude_std  -> Up
```

then publish:

```text
cov[0] = east_std^2
cov[4] = north_std^2
cov[8] = up_std^2
```

Only set a known covariance type when the corresponding values are valid.

### 4. GNSS and INS frame separation

Replace the single ambiguous frame concept with explicit parameters, at least:

```yaml
ins_frame_id: ins_link
gnss_frame_id: rtk_antenna_link
```

`NavSatFix.header.frame_id` must identify the antenna measurement location.

Robot-specific measured transforms are not hard-coded here. They belong to the vehicle description/profile.

### 5. RTK/quality status stability

Expose both raw and normalized evidence:

- raw `position_type`;
- raw/parsed `heading_type`;
- satellite count;
- standard deviations;
- configurable normalized `rtk_fixed` derived from accepted vendor solution types.

The normalized `rtk_fixed` value must not flicker simply because the current packet contains a different selector block.

### 6. INSStatus clarity

The existing `position_std` field is ambiguous because the current publisher fills it with altitude std.

The preferred contract should expose explicit position-quality components such as East/North/Up standard deviations or equivalently named fields. Any message change must update publisher/tests/docs together.

Keep raw vendor status fields available where they are useful for diagnosis.

### 7. Timestamp policy

First accepted default:

```text
timestamp_source = ros_receive
```

or equivalent existing behavior.

Preserve:

```text
gps_week
gps_time_ms
```

Do not convert GPS time to ROS/Unix absolute time until the ASENSING time definition, GPS epoch/rollover and GPS-vs-UTC leap-second behavior are verified.

R3 hardware testing must measure/record the relationship between device time metadata and ROS receive time.

### 8. Lever arm capability

The driver contract must distinguish `gnss_frame_id` from `ins_frame_id` so the physical antenna location can be represented.

A future optional lever-arm compensation path may be added only after orientation conventions are verified.

Default for this workstream:

```text
lever_arm_compensation = false
```

Do not hard-code the robot's measured lever arm into this repository.

### 9. Velocity and odometry semantics

Do not spend R1/R2 inventing a new global/local fusion architecture.

Document the current limitations of `/ins/velocity` and `/ins/odom`. If they remain published, ensure their frame labels do not falsely imply body-frame/local-odometry semantics.

A later focused workstream can define a proper ENU/NED/local-origin conversion if needed.

## Minimal Public Acceptance Contract

R2 is considered software-contract ready when downstream code can rely on these properties:

### `/ins/navsatfix`

- finite geodetic values when published as usable;
- GNSS antenna frame id;
- correct ENU covariance ordering;
- covariance validity does not flicker because of selector rotation;
- documented receive/device timestamp behavior;
- generic NavSat status is not falsely documented as equivalent to RTK fixed.

### `/ins/status`

- stable solution type across selector rotation;
- stable RTK fixed normalization across selector rotation;
- satellite count when received;
- explicit position standard deviations when received;
- heading metadata when received;
- GPS week/time metadata retained;
- raw fields remain available enough to diagnose receiver state.

## R0-R3 Development Gates

### R0 - Repository/protocol truth audit

No semantic implementation changes.

Inspect and record:

- branch/commit/package build state;
- parser packet header, lengths and checksum behavior;
- selector values currently handled;
- units/scales currently applied;
- exact `INSData` lifetime and reset behavior;
- current topic/frame/message semantics;
- current tests and missing cases;
- vendor documentation/captured evidence available locally;
- any consumers that would break if `INSStatus.msg` changes.

Deliverable:

`docs/rtk-gnss-contract/records/R00_TRUTH_AUDIT.md`

Gate: every R1/R2 change is tied to a documented defect/contract requirement.

### R1 - Parser and state correctness

Implement persistent auxiliary metadata with deterministic tests.

Required test sequences must prove:

- selector 0 data survives selector 32/1/2 packets;
- selector 32 status survives selector 0/1/2 packets;
- each selector updates only its own fields;
- RTK fixed source metadata does not reset between unrelated packets;
- GPS week/time handling does not regress existing parser framing.

Deliverable:

`docs/rtk-gnss-contract/records/R01_PARSER_STATE.md`

Gate: parser/state output is stable under alternating auxiliary blocks.

### R2 - ROS GNSS interface correctness

Implement and test:

- `ins_frame_id` / `gnss_frame_id` separation;
- NavSatFix ENU covariance;
- covariance validity handling;
- stable `/ins/status` fields;
- clearer position standard-deviation fields;
- documented timestamp-source default;
- minimal invalid/non-finite fix handling;
- explicit documentation of `/ins/pose`, `/ins/velocity`, `/ins/odom` limitations.

Deliverable:

`docs/rtk-gnss-contract/records/R02_ROS_CONTRACT.md`

Gate: runtime may integrate `/ins/navsatfix` and `/ins/status` on recorded data.

### R3 - Real receiver / vehicle validation

Use the real ASENSING device/RTK setup.

Record at least:

- serial stability and reconnect behavior;
- publish rates;
- latitude/longitude/altitude sanity;
- solution type transitions;
- RTK fixed/float behavior where available;
- satellite count behavior;
- East/North/Up standard deviations;
- static-position scatter;
- heading/orientation sanity if inspected;
- ROS receive timestamp vs GPS time metadata observations;
- GNSS outage/degradation/recovery behavior if practical;
- measured antenna frame/lever-arm documentation on the vehicle.

Deliverable:

`docs/rtk-gnss-contract/records/R03_HARDWARE_VALIDATION.md`

Gate: only after R3 may a consumer-facing stable tag/SHA be proposed for vehicle use.

## Relationship to RTAB-Map Runtime Work

This repository does not wait for the whole RTAB runtime to be built.

Parallel sequence:

```text
RTK repo:     R0 -> R1 -> R2 -> R3
                         |
                         +------ software contract usable by runtime P4

Runtime repo: P0 -> P1 -> P2 -> P3 -> P4 RTK prior -> P5 ...
```

P4 may start on bag data once R2 is accepted for the exact branch under test. Final vehicle-level RTK acceptance still requires R3 evidence.

## Validation Labels

Use:

- `STATIC_VALIDATED`
- `BAG_VALIDATED`
- `VEHICLE_VALIDATED`

Do not call a source-code inspection or parser unit test a hardware validation.

## Current Status

| Gate | State | Evidence |
| --- | --- | --- |
| Branch creation | DONE | `feat/rtabmap-gnss-contract` |
| Architecture/contract freeze | DONE | this document + `AGENTS.md` |
| R0 truth audit | NOT STARTED | none |
| R1 persistent state | BLOCKED BY R0 | none |
| R2 ROS contract | BLOCKED BY R1 | none |
| R3 hardware validation | BLOCKED BY R2 | none |
| Final SHA/tag freeze | NOT ALLOWED YET | R3 required |

## Files to Read Before Implementation

1. `AGENTS.md`
2. `RTK_GNSS_CONTRACT.md`
3. `docs/rtk-gnss-contract/CODEX_PHASE_PROMPTS.md`
4. current parser/node/message/config/tests
5. current phase record if present

Do not use navigation-runtime implementation concerns to justify protocol shortcuts here.
