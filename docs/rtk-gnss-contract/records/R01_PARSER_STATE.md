# R01 Parser / Auxiliary State Record

Status: **SOFTWARE VERIFIED**

## Defect

ASENSING auxiliary metadata is selector-driven and arrives asynchronously. The pre-R1 parser effectively emitted only the auxiliary group in the current packet, allowing unrelated values such as position std, solution type, satellite count and attitude std to return to default values on following packets.

## TDD evidence

Regression expectations were added first in commit:

```text
921f8992981a9a0b6fba7ba270b52f9112c9d32a
test(parser): cover auxiliary metadata persistence
```

Production persistence implementation:

```text
3e67157ab39f32bf35161bc0d4931ee148e75601
fix(parser): persist asynchronous ins metadata
```

The parser now begins each valid packet from the latest aggregate state, overwrites all per-main-packet measurements, then overwrites only the auxiliary fields carried by the current selector and stores the resulting state for the next packet.

## Validity model

Explicit `has_*` flags distinguish "not seen yet" from legitimate numeric zero for:

- position std;
- velocity std;
- attitude std;
- position/solution status;
- GPS week;
- temperature;
- wheel-speed status.

## Selector update boundary

| Selector | Persistent auxiliary group updated |
| --- | --- |
| `0` | latitude/longitude/altitude std |
| `1` | north/east/ground velocity std |
| `2` | roll/pitch/yaw std |
| `22` | temperature |
| `32` | position type, satellite count, heading type |
| `33` | wheel-speed status |

Per-packet main measurements such as roll/pitch/yaw, gyro/accel, geodetic position and velocity still update from every new main frame; R1 does not freeze them as stale state.

## Verification

Later full ROS 2 Humble CI checkpoints build the parser and execute the regression suite successfully, including the R1/R2/IMU integrated checkpoint and the R3 tooling descendant. The implementation is therefore software-verified, but no claim is made here about real receiver enum meanings or hardware behavior.
