# Pollen robotd in Unity

The Linux obstacle scene uses Pollen's native Rust `robotd` process for observations,
ONNX inference, action history/filtering, gains, safety, startup and fall recovery.
Unity supplies navigation commands and renders the body; native MuJoCo owns physics.
The original XR prefab retains its previous C# controller as a reference. No Unity
inference worker is created when `useRobotd` is enabled.

## Setup and use

From the project root on Linux x86_64:

```sh
python3 Tools/Robotd/prepare.py
```

This builds under the ignored `Builds/Robotd` directory, installs no system services,
and records hashes in `Builds/Robotd/provenance.json`. Rust/Cargo, Python 3.12+ and a C
compiler must be available. Open `MicroDuckObstacles` and press Play. The scene starts
its own daemon and stops that exact process when Play ends. WASD/QE and click-to-walk
work as before. Navigation retains its destination during falls. It gives the actor
up to three simulated seconds at zero velocity to get up, then permits the existing
bounded movement retries if it remains stuck. It never teleports or resets physics.

The default is 4x requested simulation speed, 200 Hz physics and 50 Hz control **in
simulated time**. Each policy cycle drives four 5 ms MuJoCo steps. Rendering does not
have to keep up with every physics step. Set `robotdLockstep` false on the scene
instance to run the unmodified reference daemon at 1x. Do not accelerate that mode.
`MICRODUCK_ROBOTD_ROOT` can point a desktop player to a prepared runtime directory;
otherwise the adapter searches parent directories for `Builds/Robotd`.

## Pins and boundary

- Daemon: `pollen-robotics/microduck` at `b2b0d213a402caeac492dd662ea891faa17f67c0`.
- Policies: `pollen-robotics/microduck-policies` at `665ae4eb20bfe13bca1ef4ea6d4bb6b4a096e448`.
  Walking is `velstand.onnx`, with no separate standing model. The other policy slots
  have explicit local paths. They use the pinned daemon's default skill timing.
- ONNX Runtime CPU: 1.23.2; archive checksum is verified by setup.
- MuJoCo: the project's existing native 3.15.0 runtime and imported MJCF.

`simulation-clock.patch` is the complete local modification to Pollen's source.
It does not rewrite the controller or recovery algorithms. `--sim-lockstep` opts
into body protocol 2; ordinary protocol 1 and hardware behavior retain the upstream
wall-clock scheduler. The original executable is kept as `robotd-upstream`.

Protocol 2 adds `tick {period_us}` and `commit` transactions. The simulator returns
an integer `time_us` and `advance`; a waiting reply is a heartbeat, not a control
cycle. Every advanced timestamp must increase by exactly one period. Unity publishes
one sensor snapshot, waits for the daemon's complete control cycle, then applies its
targets/gain/torque and steps physics. Socket threads never touch MuJoCo pointers.
A command RPC is acknowledged before releasing that cycle's sensor snapshot.

Behavioral timestamps and command expiry share simulated time. Socket deadlines,
process liveness, performance measurements and `robot.state` timestamps remain wall
clock. Explicit simulation waits keep the liveness heartbeat fresh; transport errors
do not. The hardware minimum wall-frequency health threshold is inapplicable in
lockstep mode; actual wall throughput is still reported. Closing/disconnecting a
transport causes an explicit error and preserves the goal; no automatic reset occurs.

The physical wire has 15 joints; policy inference has 14. The source model has no
mouth actuator, so that wire slot is zero and mouth targets have no physical effect,
as in Pollen's body adapter. IMU values use the trunk frame, radians and rad/s.
Gains scale native position stiffness by `kp/200`; torque off sets it to zero.
The source force/joint limits remain active. Load telemetry uses Pollen's development
simulator's `abs(actuator_force)*100` proxy, not a calibrated electrical model.

**Simulation configuration differences:** pickup detection is explicitly disabled.
Its classifier falsely paused a robot resting against a box during validation, and
its real motor-current features are not reproduced by these actuators. The original
pickup implementation and model remain available. Audio and idle head motion are
also disabled for this navigation scene. This setup is not a claim of complete
hardware or actuator-model equivalence. It does not implement training or BAM.

## Validation and logs

```sh
python3 Tools/Robotd/clock_trial.py
cargo test --locked --manifest-path Builds/Robotd/source/Cargo.toml -p duck-control -p robotd
```

The trace test runs the actual daemon with scripted sensors, compares all 300 motor
outputs at 1x and 4x, and pauses during a fall. This tests timing/control equivalence,
not physics realism. Results are written to `Builds/Robotd/validation/clock-trace.json`.

Each Play session writes `Builds/Robotd/runs/<session>/robotd.log` and its exact config.
`captureRobotdTrace` additionally records the state stream; it is off by default to
avoid unbounded per-frame logs. Full physical recovery is limited by the policy and
contact geometry: a trapped robot may keep attempting without succeeding.

### Checked on 2026-10-30 (Linux editor)

- 336 Rust tests passed (18 existing tests ignored).
- All 300 motor output packets matched exactly at 1x and 4x, including a pause
  during a fall. Pausing remained healthy; disconnecting the body became unhealthy.
- Native MuJoCo flat-ground navigation reached its target. A second trial applied
  -0.8 Nm for 0.25 simulated seconds: `limp_fall -> limp_pose -> walk` retained the
  goal, physically stood up, and reached it without a reset (15.94 simulated seconds
  in 3.983 wall seconds, approximately 4.00x).
- Pausing Unity held native time at 451.245 seconds and kept the daemon healthy.
- Difficult step/box contact remained a failure case: attempts continued, but escape
  was not guaranteed. This is not a claim that the policy can recover every pose.

Local evidence is under `Builds/Robotd/validation` (`physical-summary.json`,
`physical-trials.log`, `unity-pause.json`, `clock-trace.json`, `cargo-tests.log`).
The normal scene starts on the box; the temporary flat-ground test probe was removed.
