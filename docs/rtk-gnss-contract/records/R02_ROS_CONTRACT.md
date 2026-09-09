# R02 ROS GNSS / INS Contract Record

Status: **SOFTWARE VERIFIED; REAL HARDWARE R3 PENDING**

## Accepted downstream contract

### `/ins/navsatfix` — `sensor_msgs/msg/NavSatFix`

- GNSS measurement frame: configured `gnss_frame_id`, default `rtk_antenna_link`;
- finite/range sanity is checked before a fix is advertised usable;
- position covariance is advertised only after a valid position-std selector has been received;
- covariance diagonal follows ROS ENU:

```text
cov[0] = longitude_std²  # East
cov[4] = latitude_std²   # North
cov[8] = altitude_std²   # Up
```

Generic `NavSatFix.STATUS_FIX` is not documented as equivalent to RTK fixed.

### `/ins/status` — `agt_asensing_driver/msg/INSStatus`

Carries persisted raw/normalized quality evidence including:

- raw `ins_status`, `position_type`, `heading_type` and satellite count;
- validity flags for asynchronous quality groups;
- explicit East/North/Up position std;
- velocity and attitude std groups when valid;
- normalized configurable `rtk_fixed`;
- GPS week/time metadata;
- temperature and wheel status when available.

`position_std` is retained only as a backward-compatible scalar summary and is defined as max(E,N,U) when valid.

## Frame separation

Default configuration:

```yaml
ins_frame_id: ins_link
gnss_frame_id: rtk_antenna_link
rtk_fixed_types: [4]
```

Robot-specific measured transforms are not hard-coded into the driver.

## Timestamp policy

Accepted software default remains ROS receive time. GPS week/time remain diagnostic/device metadata. No GPS→Unix/UTC conversion was introduced because epoch/leap-second semantics for the exact receiver have not yet been proven.

## Standard IMU channel

The branch also publishes:

```text
/ins/imu
sensor_msgs/msg/Imu
```

It maps already-parsed receiver output to ROS:

- angular velocity: parsed gyro in rad/s;
- linear acceleration: parsed acceleration in m/s²;
- device fused roll/pitch/yaw orientation: **opt-in only**.

Default:

```yaml
use_device_orientation_in_imu: false
```

With orientation disabled, the message advertises orientation unavailable. This prevents a vendor north-zero/clockwise or otherwise non-REP-103 heading convention from silently becoming ROS yaw. R3 physical direction tests must precede enabling it.

## Compatibility outputs not accepted as navigation truth

- `/ins/pose`: not a complete Cartesian fused pose contract;
- `/ins/velocity`: current N/E/ground navigation-frame quantities, diagnostic/compatibility only;
- `/ins/odom`: no accepted local position origin, not an `odom -> base` source.

## TDD / implementation evidence

R2 expectation test:

```text
423960068fa0cf65746f9cc82d0b9a5010952bb2
test(gnss): define ros contract expectations
```

Pure conversion implementation:

```text
d9319c6e8954b5c389c3edc7139dc916255640cb
feat(gnss): add testable ros conversion contract
```

Humble node-integration compatibility fix:

```text
4dc4de96efe515c088197202d87068dbc2850eb1
fix(gnss): use Humble-compatible time conversion
```

Standard IMU RED test:

```text
3b58163107d73ac2bd69f255c82e98d51a12faf5
test(imu): define standard imu publication contract
```

IMU conversion implementation:

```text
e3959d4d091ca82ef390b8d68b4f8e9b9e594b27
feat(imu): add standard imu message conversion
```

Software checkpoint:

```text
95c7eaecf0f0d9dd5c3068b2780b1c90ba5efac2
```

Its ROS 2 Humble CI run built and tested successfully. R3 monitoring/tooling descendants also retain these tests.

## Remaining R3 hardware facts

Software verification does not prove:

- exact vendor RTK fixed enum;
- physical orientation axes/sign/heading zero;
- static real-world scatter;
- RTK degradation/recovery;
- reconnect behavior;
- base-to-antenna lever arm;
- correction ingress path;
- GPS vs ROS clock relationship.

Those remain in `R03_HARDWARE_VALIDATION.md`.
