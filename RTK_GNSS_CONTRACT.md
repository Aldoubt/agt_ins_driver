# RTK / GNSS Contract Workstream

This is the visible entry point for correcting and validating `agt_ins_driver` before its data is treated as a navigation/global-position prior.

## Current Branch

- Repository: `Aldoubt/agt_ins_driver`
- Branch: `feat/rtabmap-gnss-contract`
- Base at branch creation: `master@8d8ebe73ddebd6f68b6acafd02dd9f559a67d89c`
- Status: **R1/R2 + STANDARD IMU + R3 MONITORING SOFTWARE VERIFIED; REAL HARDWARE R3 PENDING**
- Final SHA/tag: **NOT FROZEN**

The branch may continue to evolve through real hardware R3. Do not lock a final consumer release until the R3 acceptance record supports it.

## Purpose

The goal is not to make this repository RTAB-specific. The goal is to make it a reliable ROS 2 GNSS/INS capability that any runtime can consume.

The first accepted downstream contract is:

```text
/ins/navsatfix
/ins/status
```

A standard `/ins/imu` channel is also available. Gyro/acceleration mapping is software-verified; receiver fused orientation is disabled by default until physical convention tests pass.

Other outputs remain compatibility/diagnostic interfaces and are not accepted as primary navigation contracts.

## Architecture boundary

The ASENSING receiver is treated as the owner of its internal GNSS/RTK + IMU/INS fusion. This driver does not run a second fusion backend over those same internal measurements.

```text
GNSS / RTK ----\
                +--> ASENSING internal INS/fusion --> position / velocity / attitude
IMU -----------/                                      + raw inertial output
                                                         |
                                                         v
                                                  agt_ins_driver
                                                         |
                    +----------------------+-------------+------------------+
                    |                      |                                |
             /ins/navsatfix          /ins/status                       /ins/imu
                    |                      |                                |
                    +----------------------+--------------------------------+
                                           |
                                           v
                                  downstream consumer
```

`Aldoubt/agt_navigation_runtime` must not copy or patch the ASENSING parser. Runtime consumes ROS interfaces and chooses how they constrain RTAB/localization/navigation.

## Accepted software properties

### Persistent auxiliary metadata

Selector-driven quality/status groups persist across unrelated packets and carry explicit validity so "not received yet" is not confused with zero. Main per-packet measurements still update every frame.

### `/ins/navsatfix`

- `header.frame_id` identifies the GNSS antenna measurement location (`rtk_antenna_link` by default);
- finite geodetic values are required before a usable fix is advertised;
- ROS covariance ordering is East/North/Up:

```text
cov[0] = longitude_std²
cov[4] = latitude_std²
cov[8] = altitude_std²
```

- covariance is not claimed known until the async position-std group is valid;
- generic `NavSatFix.STATUS_FIX` is not equivalent to RTK fixed.

### `/ins/status`

Exposes stable persisted/raw quality evidence and validity including position type, heading type, satellite count, E/N/U standard deviations and configurable normalized `rtk_fixed`.

The exact vendor meaning of solution enums is still a hardware/vendor-evidence item; default `rtk_fixed_types: [4]` is configurable rather than hidden in downstream runtime logic.

### `/ins/imu`

Maps parsed gyro and acceleration to standard `sensor_msgs/Imu`. Device fused roll/pitch/yaw is intentionally opt-in:

```yaml
use_device_orientation_in_imu: false
```

Keep it false until real physical level/left-roll/nose-up/counter-clockwise-yaw tests verify or explicitly map the receiver convention to ROS REP-103.

### Frames

```yaml
ins_frame_id: ins_link
gnss_frame_id: rtk_antenna_link
```

Robot-specific `base_link -> ins_link` and `base_link -> rtk_antenna_link` transforms belong to the vehicle description/profile, not hard-coded driver values.

### Timestamp policy

Current accepted default is ROS receive time. `gps_week` and `gps_time_ms` remain device metadata. Do not convert them to ROS absolute time until GPS epoch/rollover/GPS-vs-UTC behavior is verified.

### Lever arm

Frame separation represents the different physical measurement locations. Mathematical lever-arm compensation remains disabled/deferred until orientation convention and real installation geometry are verified.

## Compatibility outputs

Do not use these as primary runtime localization/odometry contracts yet:

```text
/ins/pose
/ins/velocity
/ins/odom
```

Current limitations:

- `/ins/pose` does not constitute an accepted complete local/global Cartesian fused pose;
- `/ins/velocity` carries north/east/ground navigation-frame quantities rather than a verified body-frame twist;
- `/ins/odom` has no accepted local position origin and must not own `odom -> base`.

## Evidence records

- `docs/rtk-gnss-contract/records/R00_TRUTH_AUDIT.md`
- `docs/rtk-gnss-contract/records/R01_PARSER_STATE.md`
- `docs/rtk-gnss-contract/records/R02_ROS_CONTRACT.md`
- `docs/rtk-gnss-contract/records/R03_HARDWARE_VALIDATION.md`

## R3 field workflow

The software tooling is implemented and Humble-build/test verified. The real hardware gate is intentionally still pending.

Field guide:

`docs/rtk-gnss-contract/R3_DRIVE_VALIDATION.md`

Codex observer prompt:

`docs/rtk-gnss-contract/R3_CODEX_FIELD_PROMPT.md`

Typical standalone run:

```bash
ros2 run agt_asensing_driver r3_drive_validation.sh \
  --label greenhouse_rtk_01 \
  --output-root ~/agt_r3_runs \
  --static-window 60
```

If another runtime already owns the sensor serial device:

```bash
ros2 run agt_asensing_driver r3_drive_validation.sh \
  --no-driver \
  --label greenhouse_rtk_runtime_01
```

The workflow is observation-only: it starts sensor/monitor/rosbag processes, never vehicle motion commands. `report.json` and `report.md` update during the drive so a local Codex session can inspect them without stopping acquisition. The automatic report always remains `EVIDENCE_ONLY` until the human/manual R3 checks are reviewed.

## Required real hardware gate

R3 must record at least:

- serial stability and reconnect behavior while safely stationary;
- topic publish rates and data gaps;
- latitude/longitude/altitude sanity;
- raw solution-type and RTK fixed/float/degraded transitions;
- satellite count and E/N/U std behavior;
- static-position scatter;
- physical roll/pitch/yaw axis/sign/zero tests before orientation is enabled;
- ROS receive stamp vs GPS week/time behavior;
- controlled GNSS/RTK degradation/recovery where practical;
- measured antenna lever arm / frame geometry;
- actual deployed correction ingress path.

Only after review of that evidence may a consumer-facing candidate SHA/tag be proposed.

## Current Status

| Gate | State | Evidence |
| --- | --- | --- |
| Branch creation | DONE | `feat/rtabmap-gnss-contract` |
| Architecture/contract boundary | DONE | this document + `AGENTS.md` |
| R0 repository/source truth audit | AUDITED | `R00_TRUTH_AUDIT.md` |
| R1 persistent async state | SOFTWARE VERIFIED | regression tests + `R01_PARSER_STATE.md` |
| R2 ROS GNSS/status contract | SOFTWARE VERIFIED | conversion tests + `R02_ROS_CONTRACT.md` |
| Standard `/ins/imu` mapping | SOFTWARE VERIFIED | RED→GREEN contract tests; orientation remains hardware-gated |
| R3 metric/monitor tooling | SOFTWARE VERIFIED | metric RED→GREEN tests + Humble build/test of field monitor |
| R3 real receiver/vehicle validation | NOT RUN | `R03_HARDWARE_VALIDATION.md` remains NOT RUN |
| Final SHA/tag freeze | NOT ALLOWED YET | hardware R3 required |

## Relationship to RTAB-Map runtime work

The RTK repository does not wait for the whole runtime to be built:

```text
RTK repo:     R0 -> R1 -> R2 -> R3 hardware
                         |
                         +------ software contract available to runtime P4

Runtime repo: P0 -> P1 -> P2 -> P3 -> P4 RTK prior -> P5 ...
```

Runtime may integrate the software-verified `/ins/navsatfix` + `/ins/status` contract on the exact branch under test. Final vehicle-level acceptance still requires R3 evidence.

## Validation labels

Use labels precisely:

- `SOFTWARE VERIFIED` — build/unit/contract evidence only;
- `STATIC_VALIDATED` — real receiver static hardware evidence;
- `BAG_VALIDATED` — recorded/replayed data evidence;
- `VEHICLE_VALIDATED` — controlled real vehicle test evidence.

Never relabel software/CI evidence as vehicle validation.

## Files to read before further work

1. `AGENTS.md`
2. `RTK_GNSS_CONTRACT.md`
3. current phase record
4. `docs/rtk-gnss-contract/R3_DRIVE_VALIDATION.md` for hardware testing
5. `docs/rtk-gnss-contract/R3_CODEX_FIELD_PROMPT.md` for local Codex observation

Do not use navigation-runtime implementation concerns to justify protocol shortcuts here.
