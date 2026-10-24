# Motor test missions

Run one motor test at a time:

```sh
cargo run -- run motor-y-forward
```

Each mission sends a `LOCAL` speed command with only one nonzero component
at magnitude 0.5 by default, and runs until killed using the existing
mission shutdown behavior.

| Missions | LOCAL component |
| --- | --- |
| `motor-x-right`, `motor-x-left` | x positive, negative |
| `motor-y-forward`, `motor-y-back` | y positive, negative |
| `motor-z-up`, `motor-z-down` | z positive, negative |
| `motor-pitch-positive`, `motor-pitch-negative` | x rotation positive, negative |
| `motor-roll-positive`, `motor-roll-negative` | y rotation positive, negative |
| `motor-yaw-cw`, `motor-yaw-ccw` | z rotation positive (clockwise), negative (counterclockwise) |

Axes are +Y forward, +X right, +Z up, and positive yaw clockwise.
Rotation components follow the control board's protocol order (pitch, roll,
yaw). `motor-yaw-positive` and `motor-yaw-negative` remain aliases for the
clockwise and counterclockwise missions.

All 12 missions share these settings in `config.toml`:

```toml
[missions.motor_test]
speed = 0.5
```

Speed must be finite, greater than zero, and at most one. There is no duration
limit; remove `duration_seconds` from older configs. Existing configs without
this section use the default shown above. The checked-in vehicle motor matrix is all zeros; it
must be configured for the vehicle before these tests can drive its motors.
