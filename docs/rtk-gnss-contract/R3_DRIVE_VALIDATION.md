# R3 Drive Validation Workflow

This workflow is for real ASENSING RTK/INS hardware. The operator drives the vehicle; the software only observes, records and reports.

## Safety boundary

The R3 tools in this repository never publish vehicle motion commands and never change receiver configuration. Do not disconnect an antenna, correction link or serial cable while the vehicle is moving. Any reconnect/degradation manipulation must be performed while stationary and safe.

## What is recorded

The monitor subscribes to:

- `/ins/navsatfix` — accepted GNSS position contract;
- `/ins/status` — accepted RTK/quality contract;
- `/ins/imu` — standard IMU channel;
- `/ins/velocity` — diagnostic-only north/east velocity evidence; this does not promote the topic to an accepted navigation contract;
- `/ins/r3/marker` — optional event annotations.

The bag also records `/tf_static`.

The monitor continuously overwrites `report.json` and `report.md`, so Codex or another observer can inspect the current run without stopping it. On SIGINT/shutdown the same files are written one final time.

Reported evidence includes:

- message count, receive rate, maximum receive gap and header timestamp regressions for GNSS/status/IMU/diagnostic velocity;
- usable GNSS fix count;
- RTK-fixed ratio based on valid persisted solution-status samples;
- `position_type` histogram and fixed/non-fixed transitions;
- satellite count and East/North/Up standard deviations;
- trajectory offsets from the first usable fix;
- an initial stationary scatter window starting at the first usable fix;
- IMU gyro norm, acceleration norm and count of samples that advertise orientation;
- GPS week/time continuity and regression count;
- optional markers and automatically detected solution transitions.

The automatic report is intentionally labeled `EVIDENCE_ONLY`. It cannot promote the branch to R3 PASS or READY_FOR_TAG.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agt_asensing_driver
source install/setup.bash
colcon test --packages-select agt_asensing_driver
colcon test-result --verbose
```

## One-command field run

If this script should own the ASENSING serial driver:

```bash
ros2 run agt_asensing_driver r3_drive_validation.sh \
  --label greenhouse_rtk_01 \
  --output-root ~/agt_r3_runs \
  --static-window 60
```

If another runtime already owns the ASENSING serial device, do not start a second driver:

```bash
ros2 run agt_asensing_driver r3_drive_validation.sh \
  --no-driver \
  --label greenhouse_rtk_runtime_01
```

A run directory is created as:

```text
~/agt_r3_runs/YYYYMMDD_HHMMSS_<label>/
├── session.txt
├── driver.log          # when this workflow starts the driver
├── monitor.log
├── bag.log
├── report.json         # live snapshot + final machine-readable evidence
├── report.md           # live snapshot + final human-readable evidence
└── bag/
    └── metadata.yaml + rosbag storage
```

Press `Ctrl-C` only after the vehicle is stopped and the run is complete. The script stops rosbag first, then the monitor so the final report can include all observed samples, then the driver if it owns it.

## Operator sequence

1. Start the workflow with the vehicle stationary in the best available open-sky area.
2. Remain stationary for the configured initial static window after the first usable fix. Default: 60 s.
3. Drive a controlled low-risk route including straight motion and turns.
4. If practical, include a safe GNSS/RTK quality degradation and recovery segment by changing environment/corrections under controlled conditions. Do not manipulate hardware while moving.
5. Stop the vehicle.
6. If reconnect behavior must be checked, perform the serial/driver reconnect only now while stationary.
7. Stop the workflow with `Ctrl-C`.
8. Review `report.md`, `report.json`, rosbag metadata and logs before updating `R03_HARDWARE_VALIDATION.md`.

## Optional markers

Codex or an observer can annotate the run without changing the data path:

```bash
ros2 topic pub --once /ins/r3/marker std_msgs/msg/String \
  "{data: 'STATIC_OPEN_SKY_START'}"
```

Examples:

```text
STATIC_OPEN_SKY_START
DRIVE_START
RTK_DEGRADE_START
RTK_RECOVERY
VEHICLE_STOPPED
SERIAL_RECONNECT_START
SERIAL_RECONNECT_COMPLETE
```

Markers are evidence annotations only.

## Required manual checks

The automated monitor cannot prove these by itself:

- ASENSING roll/pitch/yaw axes, signs and heading zero-direction relative to ROS REP-103;
- the vendor meaning of each `position_type`, especially the RTK-fixed enum;
- safe RTK degradation/recovery behavior;
- serial reconnect semantics after a real disconnect;
- the measured `base_link -> rtk_antenna_link` lever arm.

Until those are reviewed, keep `use_device_orientation_in_imu: false` and keep the final SHA/tag unfrozen.
