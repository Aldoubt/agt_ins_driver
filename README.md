# agt_ins_driver

ROS 2 Humble INS/GNSS abstraction driver. The first supported device is ASENSING INS.

## Active GNSS/RTK Contract Work

The active correctness and validation workstream is isolated on:

```text
feat/rtabmap-gnss-contract
```

Start here before modifying parser or ROS GNSS semantics:

- [`RTK_GNSS_CONTRACT.md`](RTK_GNSS_CONTRACT.md) — current defects, accepted public contract, R0-R3 gates and freeze policy.
- [`AGENTS.md`](AGENTS.md) — hard rules for Codex/agentic sensor-driver development.
- [`docs/rtk-gnss-contract/CODEX_PHASE_PROMPTS.md`](docs/rtk-gnss-contract/CODEX_PHASE_PROMPTS.md) — R0-R3 prompts to execute one gate at a time.

The branch SHA is intentionally **not frozen** until real-data/hardware acceptance supports a stable consumer version.

## Build and run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch agt_asensing_driver asensing.launch.py
```

The driver currently publishes:

| Topic | Type | Contract status in the active workstream |
| --- | --- | --- |
| `/ins/navsatfix` | `sensor_msgs/NavSatFix` | target accepted GNSS position contract after R2 |
| `/ins/status` | `agt_asensing_driver/INSStatus` | target accepted GNSS/RTK quality contract after R2 |
| `/ins/pose` | `geometry_msgs/PoseStamped` | compatibility/experimental until frame semantics are accepted |
| `/ins/velocity` | `geometry_msgs/TwistStamped` | compatibility/experimental; navigation-frame velocity semantics must be documented |
| `/ins/odom` | `nav_msgs/Odometry` | compatibility/experimental; not an accepted local odometry source yet |

The driver owns ASENSING protocol parsing and ROS GNSS/INS semantic correctness. Downstream navigation systems choose how to use the standard outputs:

```text
ASENSING receiver
      |
      v
agt_ins_driver
      |
      +-- /ins/navsatfix
      +-- /ins/status
      +-- diagnostic/compatibility INS outputs
      |
      v
consumer-selected localization / mapping / navigation backend
```

`agt_ins_driver` does not own RTAB-Map, FAST-LIO2, GTSAM, Nav2, HMI or mission logic. Those integrations remain in their respective repositories.

RTK-fixed solution types are configurable with `rtk_fixed_types` in `config/asensing.yaml`, while the raw solution type remains important diagnostic evidence.

The serial protocol parser is independent of ROS and retains the current ASENSING frame header, offsets, lengths, scaling and XOR checksum behavior unless a protocol change is verified and covered by tests.
