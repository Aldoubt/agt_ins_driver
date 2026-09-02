# agt_ins_driver

ROS 2 Humble INS/GNSS abstraction driver. The first supported device is ASENSING INS.

## Active GNSS/RTK Contract Work

Active branch:

```text
feat/rtabmap-gnss-contract
```

Current state:

- R1 parser auxiliary-state persistence: **SOFTWARE VERIFIED**;
- R2 `/ins/navsatfix` + `/ins/status` contract: **SOFTWARE VERIFIED**;
- standard `/ins/imu`: **SOFTWARE VERIFIED**, device fused orientation remains opt-in pending real physical convention checks;
- R3 field monitoring/tooling: **SOFTWARE VERIFIED**;
- R3 real receiver/vehicle acceptance: **PENDING**;
- final consumer SHA/tag: **NOT FROZEN**.

Start here:

- [`RTK_GNSS_CONTRACT.md`](RTK_GNSS_CONTRACT.md) — current contract, gates and freeze policy;
- [`AGENTS.md`](AGENTS.md) — hard rules for Codex/agentic sensor-driver development;
- [`docs/rtk-gnss-contract/R3_DRIVE_VALIDATION.md`](docs/rtk-gnss-contract/R3_DRIVE_VALIDATION.md) — one-command field monitoring and rosbag workflow;
- [`docs/rtk-gnss-contract/R3_CODEX_FIELD_PROMPT.md`](docs/rtk-gnss-contract/R3_CODEX_FIELD_PROMPT.md) — read-only Codex field-observer prompt;
- [`docs/rtk-gnss-contract/records/R03_HARDWARE_VALIDATION.md`](docs/rtk-gnss-contract/records/R03_HARDWARE_VALIDATION.md) — durable hardware acceptance record;
- [`docs/rtk-gnss-contract/CODEX_PHASE_PROMPTS.md`](docs/rtk-gnss-contract/CODEX_PHASE_PROMPTS.md) — R0-R3 implementation/review prompts.

The branch SHA is intentionally **not frozen** until real-data/hardware R3 evidence supports a stable consumer version.

## Build and run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agt_asensing_driver
source install/setup.bash
colcon test --packages-select agt_asensing_driver
colcon test-result --verbose

ros2 launch agt_asensing_driver asensing.launch.py
```

## Published interfaces

| Topic | Type | Contract status |
| --- | --- | --- |
| `/ins/navsatfix` | `sensor_msgs/NavSatFix` | R2 software-verified GNSS position contract; real receiver R3 pending |
| `/ins/status` | `agt_asensing_driver/INSStatus` | R2 software-verified GNSS/RTK quality contract; vendor enum verification remains R3 evidence |
| `/ins/imu` | `sensor_msgs/Imu` | gyro/accel software-verified; device fused orientation disabled by default until physical convention R3 checks |
| `/ins/pose` | `geometry_msgs/PoseStamped` | compatibility/experimental; orientation-only today, not an accepted full pose contract |
| `/ins/velocity` | `geometry_msgs/TwistStamped` | compatibility/diagnostic; current north/east/ground navigation-frame semantics are not body-frame velocity |
| `/ins/odom` | `nav_msgs/Odometry` | compatibility/experimental; no accepted local position origin, not a runtime odometry source |

Default relevant parameters:

```yaml
ins_frame_id: ins_link
gnss_frame_id: rtk_antenna_link
rtk_fixed_types: [4]
use_device_orientation_in_imu: false
```

`rtk_fixed_types` is configurable because the exact ASENSING solution enum must be checked against evidence for the real device. Do not equate generic `NavSatFix.STATUS_FIX` with RTK fixed.

`use_device_orientation_in_imu` stays false until real level/roll/pitch/yaw tests verify or explicitly map the vendor orientation convention to ROS REP-103.

## Field validation

If this package should own the ASENSING serial driver during the test:

```bash
ros2 run agt_asensing_driver r3_drive_validation.sh \
  --label greenhouse_rtk_01 \
  --output-root ~/agt_r3_runs \
  --static-window 60
```

If the navigation runtime already owns the serial device, do **not** open it twice:

```bash
ros2 run agt_asensing_driver r3_drive_validation.sh \
  --no-driver \
  --label greenhouse_rtk_runtime_01
```

The workflow never sends robot motion commands. It runs the read-only monitor and rosbag recording and continuously writes `report.json` plus `report.md`. Keep the vehicle stationary for the initial sampling window, wait for RTK quality to stabilize before beginning the controlled drive, and stop the vehicle before any deliberate reconnect or hardware manipulation.

## Repository boundary

The receiver performs its own INS/GNSS fusion; this package parses and standardizes the receiver output. It does not add a second RTK+IMU fusion backend.

```text
ASENSING receiver
  internal GNSS/RTK + IMU/INS fusion
          |
          v
agt_ins_driver
          |
          +-- /ins/navsatfix
          +-- /ins/status
          +-- /ins/imu
          +-- compatibility/diagnostic outputs
          |
          v
consumer-selected mapping / localization / navigation backend
```

`agt_ins_driver` does not own RTAB-Map, FAST-LIO2, GTSAM, Nav2, HMI, mission logic, robot-specific static transforms or chassis safety logic.

The serial protocol parser is independent of ROS and retains verified frame header, offsets, lengths, scaling and XOR behavior unless a protocol change is supported by evidence and regression tests.
