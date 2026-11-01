# Pollen control in Unity

Unity calls `libmicroduck_control.so` directly. Normal project use needs the checked-in Linux x86-64 or Android ARM64 plugins, ONNX Runtime and policy assets; it does not run robotd, Python, a socket server or a Rust build.

The source comes from [Pollen's microduck repository](https://github.com/pollen-robotics/microduck/tree/b2b0d213a402caeac492dd662ea891faa17f67c0), pinned to `b2b0d213a402caeac492dd662ea891faa17f67c0`. `prepare.py` exports that Git commit, ignoring any working-tree changes. Generated source stays under ignored `Builds/PollenControl/source`.

The native build compiles Pollen's `robotd/src/control.rs` and `posture.rs` unchanged, and reuses `duck-control` for observations, ONNX inference, joint action mapping, recurrent state, trained action filtering, fall prediction and safety. It extracts the original startup homing, fall limp/landing/pose, command smoothing, seated startup, pause/history and target-selection blocks from `robotd/src/main.rs`. Every extracted fragment has its original source range and hash in `Builds/PollenControl/provenance.json`.

This is the navigation configuration of Pollen's controller, not the entire robot daemon. The host supplies fresh simulated sensors and velocity requests at 50 Hz, fixed 7.4 V supply, and enabled control. Pickup detection, idle animation, audio, physical devices, process shutdown, model hot-swaps and IPC are excluded. The original skill scheduler and six configured policy networks remain in the controller; this velocity-only ABI exposes automatic seated/fall recovery, not explicit kick, ground-pick or sit commands. Walking uses Pollen v8 `velstand.onnx`, including its learned get-up behavior. No code resets the physical robot after a fall.

`microduck_control.h` defines the complete ABI, joint/coordinate conventions and clock contract. ABI 1 accepts 15 joint positions, velocities and currents, trunk gyro/gravity, a scalar-first quaternion, and body-relative velocity. It returns 15 joint targets, P gain, torque, state and independent fallen telemetry. Unity maps the 14 simulated joints around the mouth slot. `md_reset` restarts the control episode only for explicit placement. Paused physics makes no native calls; accelerated physics keeps the same 20 ms simulation ticks.

## Rebuilding

The consuming Unity project does not need these steps. Developers changing the native controller need Rust 1.89 or later, the Rust Android target and Unity's Android NDK. The included Linux artifact requires glibc 2.34 or newer; Android targets API 29 with 16 KiB ELF segment alignment. Android compilation and packaging are checked separately from execution on a Quest.

```bash
# A local Pollen checkout is only needed when regenerating the native plugin.
git clone https://github.com/pollen-robotics/microduck.git /tmp/pollen-microduck
python3 Tools/PollenControl/prepare.py --upstream /tmp/pollen-microduck
rustup target add aarch64-linux-android
python3 Tools/PollenControl/build.py --ndk /path/to/Unity/Editor/Data/PlaybackEngines/AndroidPlayer/NDK
python3 Tools/PollenRuntime/prepare.py --verify \
  --core-linux Builds/PollenControl/artifacts/linux-x86_64/libmicroduck_control.so \
  --core-android Builds/PollenControl/artifacts/android-arm64/libmicroduck_control.so
```

`build.py --platform linux` builds only Linux. `--offline` uses cached Cargo dependencies. The original upstream lockfile seeds dependency resolution; the generated lockfile hash, toolchain and built artifact hashes are recorded in provenance. Hardware serial dependencies and the TCP backend are disabled through Cargo features. The explicit ORT path adapter initializes the packaged runtime without mutating Unity's process environment.

## Validation

Upstream tests run against the actual extracted modules, including the ONNX fixtures:

```bash
ORT_DYLIB_PATH="$PWD/Assets/Plugins/Linux/x86_64/libonnxruntime.so" \
  cargo test --manifest-path Builds/PollenControl/source/Cargo.toml \
  -p microduck-control -p duck-control --no-default-features --lib -- --include-ignored
```

`validate.py` exercises the actual C ABI: startup, requested forward movement, a scripted fall, limp/pose/policy handoff, seated startup, non-finite input rejection, clock validation and deterministic reset/pause replay. It can compare all 15 targets, gain and torque against a real robotd lockstep trace. That is a control-equivalence test over scripted observations, not a physics or hardware locomotion benchmark.

```bash
# Use a directory containing the staged original .onnx files and manifest.json.
python3 Tools/PollenControl/validate.py \
  --library Assets/Plugins/Linux/x86_64/libmicroduck_control.so \
  --models /path/to/staged/policies \
  --ort Assets/Plugins/Linux/x86_64/libonnxruntime.so \
  --output Builds/PollenControl/validation.json
```

For the optional daemon comparison, an existing desktop adapter installation must supply `bin/robotd`, `policies` and `onnxruntime-linux-x64-1.23.2`. `daemon_trial.py --runtime /path/to/Builds/Robotd --output Builds/PollenControl/daemon-trace.json` runs its original 1x/4x scenario using temporary local sockets. Pass that result to `validate.py --daemon-trace Builds/PollenControl/daemon-trace.json`. This test harness is never shipped or invoked by Unity.

Pollen's source is Apache-2.0; the upstream license is retained in the generated source and the packaged notices under `Assets/StreamingAssets/NativeLicenses/PollenControl`.
