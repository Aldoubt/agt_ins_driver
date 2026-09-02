# R03 Hardware Validation Record

Status: **NOT RUN — HARDWARE EVIDENCE REQUIRED**

This file is the durable R3 acceptance record. Do not mark it PASS from CI, simulation, rosbag replay alone, or the monitor's automatic `EVIDENCE_ONLY` label.

## Test identity

- Date/time:
- Operator:
- Site:
- Repository branch: `feat/rtabmap-gnss-contract`
- HEAD:
- Device/model/serial if available:
- Driver port/baud:
- RTK correction source:
- Run directory:
- Rosbag path:
- Rosbag metadata/duration:

## Physical installation

- `base_link -> ins_link` transform:
- `base_link -> rtk_antenna_link` lever arm (x/y/z, metres):
- Antenna orientation/placement notes:

## Software evidence

- Clean build:
- Test result:
- CI run/commit:
- `use_device_orientation_in_imu` during test:

## Topic health

| Topic | Count | Rate Hz | Max receive gap s | Header regressions | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| `/ins/navsatfix` | | | | | |
| `/ins/status` | | | | | |
| `/ins/imu` | | | | | |
| `/ins/velocity` diagnostic-only | | | | | |

## Static open-sky window

- Window duration:
- Usable fix count:
- East scatter/std:
- North scatter/std:
- Altitude scatter/std:
- Satellite count:
- E/N/U reported std:
- Observed solution types:
- RTK fixed ratio:

## Controlled motion

- Route description:
- Latitude/longitude motion direction plausible:
- Diagnostic N/E velocity direction plausible:
- Timestamp monotonicity:
- Data gaps/dropouts:

## RTK degradation/recovery

- Tested: YES / NO
- Method/environment:
- Fixed -> degraded transition evidence:
- Degraded -> recovered/fixed evidence:
- `position_type` transitions:
- Std/satellite behavior:

## IMU physical convention checks

Keep device orientation disabled in `/ins/imu` until all required checks below are directly observed.

| Physical action | Expected ROS interpretation | Observed | PASS/FAIL/NOT TESTED |
| --- | --- | --- | --- |
| Sensor/vehicle level | roll≈0, pitch≈0 | | |
| Left roll | verify ROS roll sign | | |
| Nose up | verify ROS pitch sign | | |
| Counter-clockwise yaw | verify ROS yaw sign/zero convention | | |
| Stationary | gyro near expected bias/noise; accel magnitude near gravity | | |

Orientation decision:

- [ ] Keep `use_device_orientation_in_imu=false`
- [ ] Vendor convention verified/mapped; orientation may be enabled in a follow-up reviewed commit

## Device time

- GPS week/time valid sample count:
- Device-time regressions:
- Device elapsed vs ROS/test elapsed:
- GPS epoch/UTC behavior proven: YES / NO

Do not switch production stamping to GPS absolute time unless the epoch/leap-second behavior is proven.

## Serial reconnect

- Tested while stationary: YES / NO
- Disconnect method:
- Publication gap:
- Recovery observed:
- Auxiliary validity after reconnect:

## Defects found

| Defect | Severity | Evidence | Fix commit | Retest |
| --- | --- | --- | --- | --- |
| | | | | |

## Acceptance

- R1 parser persistence: software evidence reviewed / pending
- R2 GNSS contract: software evidence reviewed / pending
- `/ins/imu`: software evidence reviewed / pending
- R3 real hardware: PASS / FAIL / PARTIAL / NOT RUN

Freeze recommendation: **NOT_READY**

Required reviewer notes:

- Do not equate generic NavSatFix `STATUS_FIX` with RTK fixed.
- Do not promote `/ins/odom` to runtime odometry from this test.
- Do not promote device orientation until physical convention checks pass.
- Do not freeze a SHA/tag until hardware evidence and unresolved defects are reviewed.
