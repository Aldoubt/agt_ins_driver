# R1 RED Evidence

Phase: R1 persistent parser/state correctness

The production parser is intentionally unchanged at this point.

New tests in `agt_asensing_driver/test/test_asensing_parser.cpp` require:

- position std metadata persists when selector changes to 32/1/2;
- solution metadata persists when selector changes away from 32;
- velocity and attitude std validity is explicit;
- GPS week persists after a valid extended frame;
- temperature and wheel-speed metadata validity is explicit.

Expected RED behavior on the pre-R1 parser:

- compilation fails for validity fields that do not exist yet (`has_position_std`, `has_velocity_std`, `has_attitude_std`, `has_gps_week`, `has_temperature`, `has_wheel_speed_status`), and/or
- persistence assertions fail because each packet currently starts from a fresh `INSData`.

No production parser code has been changed in this RED commit sequence.
