# agt_ins_driver

ROS 2 Humble INS abstraction driver. The first supported device is ASENSING INS.

## Build and run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch agt_asensing_driver asensing.launch.py
```

The driver publishes `/ins/navsatfix`, `/ins/pose`, `/ins/velocity`, and `/ins/status`.
The serial protocol parser is independent of ROS and retains the original ASENSING
frame header, offsets, lengths, and XOR checksums.
