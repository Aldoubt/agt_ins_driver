# R00 Repository / Protocol Truth Audit

Status: **AUDITED FOR CURRENT SOFTWARE WORKSTREAM**

This record captures source truth used by R1/R2. It is not vendor-protocol certification and is not hardware validation.

## Baseline

- Repository: `Aldoubt/agt_ins_driver`
- Workstream branch: `feat/rtabmap-gnss-contract`
- Branch base: `master@8d8ebe73ddebd6f68b6acafd02dd9f559a67d89c`
- ROS target: Humble / Ubuntu 22.04

## Source ownership

`agt_asensing_driver` owns:

- ASENSING serial framing/parsing;
- latest parsed INS/GNSS data state;
- ROS standardization of GNSS/status/IMU outputs.

It does not own RTAB-Map/Nav2/GTSAM/robot-localization fusion, vehicle motion control or robot-specific static transforms.

## Parser facts carried into R1

Implemented framing at the start of this workstream:

- header bytes: `0xbd 0xdb 0x0b`;
- main packet length: 58 bytes;
- extended length considered by the parser: 63 bytes;
- XOR checksum behavior retained rather than redesigned;
- main fields include roll/pitch/yaw, gyro, acceleration, geodetic position, N/E/ground velocity and `ins_status`;
- selector-driven auxiliary groups handled include 0, 1, 2, 22, 32 and 33;
- GPS time counter is parsed from the main frame; GPS week is retained from the validated extension.

The original correctness defect was data lifetime, not a reason to rewrite byte offsets/scales: asynchronous selector metadata was emitted with a freshly defaulted state and therefore unrelated auxiliary values could disappear on the next selector packet.

## ROS-contract defects identified

The baseline source exposed these correctness/semantic issues:

1. Auxiliary standard deviations/status fields could reset between selectors.
2. One generic frame was used for INS and GNSS measurement locations.
3. `NavSatFix` covariance diagonal did not follow ROS ENU ordering.
4. Covariance could be advertised before the asynchronous std group was known valid.
5. `rtk_fixed` depended on selector-32 metadata that could disappear between packets.
6. `INSStatus.position_std` represented altitude std rather than an explicit position-quality contract.
7. `/ins/velocity` contains north/east/ground quantities but had ambiguous frame meaning.
8. `/ins/odom` has no defined local position origin and cannot be treated as accepted `odom -> base` output.
9. `/ins/pose` is not a complete fused Cartesian pose; current compatibility output primarily exposes orientation.
10. ROS messages use receive time while GPS week/time are retained separately.

## Unknown / intentionally unproven vendor semantics

Source alone does not prove:

- exact meaning of every `position_type` / `heading_type` numeric value;
- that configured type `4` means RTK fixed for the exact receiver under test;
- GPS epoch/UTC/leap-second behavior needed for safe GPS→ROS absolute timestamp conversion;
- whether the receiver orientation uses ROS REP-103 axes/signs/zero heading;
- how RTK corrections enter the physical receiver in the deployed setup;
- the real vehicle `base_link -> rtk_antenna_link` lever arm.

These are R3/vendor-evidence items and were not guessed into the software contract.

## R1/R2 decision

R1 therefore changes state lifetime only while preserving known parser framing/scaling behavior. R2 standardizes GNSS/status semantics and separates frames without inventing a new fusion architecture. `/ins/navsatfix` and `/ins/status` are the first accepted downstream interfaces; compatibility pose/velocity/odom remain outside the accepted navigation contract.
