# agt_ins_driver

ROS 2 Humble INS abstraction driver. The first supported device is ASENSING INS.

## Build and run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch agt_asensing_driver asensing.launch.py
```

The driver supports ASENSING INS on ROS 2 Humble and publishes:

| Topic | Type |
| --- | --- |
| `/ins/navsatfix` | `sensor_msgs/NavSatFix` |
| `/ins/pose` | `geometry_msgs/PoseStamped` |
| `/ins/velocity` | `geometry_msgs/TwistStamped` |
| `/ins/odom` | `nav_msgs/Odometry` |
| `/ins/status` | `agt_asensing_driver/INSStatus` |
| `/ins/raw_frame` | `std_msgs/UInt8MultiArray` |

The status message retains GPS week/time, temperature, wheel-speed status,
solution types, satellite count, and standard deviations. According to the
ASENSING protocol, position types `48`, `49`, and `50` are fixed solutions;
these are configured by default through `rtk_fixed_types` in
`config/asensing.yaml`.

The intended integration path is:

```text
agt_ins_driver -> robot_localization -> GTSAM GPSFactor -> FAST-LIO2 global optimization
```

The serial protocol parser is independent of ROS and retains the original ASENSING
frame header, offsets, lengths, and XOR checksums.

## rosbag data collection

`/ins/raw_frame` contains each validated ASENSING frame as raw bytes. A normal
frame has 58 bytes; a frame with the GPS-week extension has 63 bytes. The topic
is suitable for comparing the ROS2 parser with the original ROS1 driver or the
vendor upper computer.

Record raw frames together with the decoded data:

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash

ros2 bag record \
  /ins/raw_frame \
  /ins/status \
  /ins/navsatfix \
  /ins/pose \
  /ins/velocity \
  /ins/odom \
  /rosout \
  -o asensing_check
```

Inspect and replay the recording:

```bash
ros2 bag info asensing_check
ros2 bag play asensing_check
ros2 topic echo /ins/raw_frame
```

When reviewing a bag, compare `num_satellite`, `position_type`,
`rtk_fixed`, the GPS time, and the raw frame bytes. Position types `48`, `49`,
and `50` are fixed solutions; `16` is single-point positioning.

## ROS1 reference driver

The original ROS1 driver is included at:

```text
ASENSING_INS_ROS1_Driver_V1.02(1)/ASENSING_INS_Driver_V1.02/
```

The ROS1 node is:

```text
ASENSING_INS_ROS1_Driver_V1.02(1)/ASENSING_INS_Driver_V1.02/src/ASENSING_INS_node.cpp
```

Its message definition and protocol notes are in `msg/ASENSING.msg` and the
PDF under the same ROS1 directory. The ROS1 package publishes the combined
`ASENSING_INS` message; the ROS2 package publishes standard ROS messages plus
`/ins/status` and `/ins/raw_frame`.
