# Robot control

The Linux obstacle scene now runs the actual Pollen `robotd` process. Setup, the
clock patch, simulator configuration and current validation are documented in
[Tools/Robotd/README.md](../../../Tools/Robotd/README.md). No C# policy worker or
fall state machine runs on that path.

## Earlier C# reference implementation

The remaining notes describe the reference path retained for the original XR prefab.

Source: pollen-robotics/microduck commit
`b2b0d213a402caeac492dd662ea891faa17f67c0` (inspected 2026-10-30).
The C# port is distributed with `Pollen-LICENSE.txt` (Apache-2.0).

- [Fall predictor](https://github.com/pollen-robotics/microduck/blob/b2b0d213a402caeac492dd662ea891faa17f67c0/duck-control/src/fall.rs): projected gravity derivative `-(gyro × gravity)`, 300 ms prediction, -0.90 tilt gate, -0.5 predicted threshold, 60 ms debounce, and 200 ms upright arming. A debounced fallen report does not disable the actor.
- [Landing and pose sequence](https://github.com/pollen-robotics/microduck/blob/b2b0d213a402caeac492dd662ea891faa17f67c0/robotd/src/main.rs): follow measured joints at low gain; wait for gyro magnitude below 1 rad/s for 200 ms, capped at 1.5 seconds; ramp from the actual landed joint angles to home over 600 ms; hand back even if still down. Rearm only after standing again.
- [Policy output](https://github.com/pollen-robotics/microduck/blob/b2b0d213a402caeac492dd662ea891faa17f67c0/robotd/src/control.rs): raw actions feed observation history; scale 0.9 for walking/combined velstand, 1.0 for a separate standing model; filter head targets with alpha 0.5 and leg targets with alpha 0.7. Actor history and target-filter anchors restart at hand-back, matching `Controller::reset`.
- [Deployment defaults](https://github.com/pollen-robotics/microduck/blob/b2b0d213a402caeac492dd662ea891faa17f67c0/deploy/robotd.toml): combined velstand is the walking network with no separate standing model. Equal model references represent that configuration in the Unity scene.

The 61 observations and 14 actions omit the mouth, matching Pollen's observation
contract. Physical integration remains 200 Hz and control/inference 50 Hz.

Pollen's Dynamixel gains 50/160/200 are mapped to relative native position-actuator
stiffness (0.25/0.8/1.0). This is a simulator adaptation, not an exact reproduction
of Dynamixel electronics or the training environment's BAM actuator model.
The source MJCF's force limits and joint constraints remain active.

The landing sequence changes motor targets and gains only. It never changes
qpos/qvel, the root anchor, contacts, or the native simulation clock.
Navigation pauses during the finite limp/pose sequence, retains its destination,
then replans using measured feet. A fallen verdict never prevents policy driving.
NavMesh planning and waypoint steering are application code, not Pollen features.

Regression cases are in `Tools/Tests/PollenControlTrials`; run with a .NET 8 SDK:

```sh
dotnet run --project Tools/Tests/PollenControlTrials --configuration Release
```
