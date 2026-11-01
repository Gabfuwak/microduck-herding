# Pollen runtime assets

The Unity project includes the native libraries and raw models needed for Linux
x86_64 and Android ARM64. Opening the project on Linux or building the Quest app
does not require downloading models, installing Python packages, or running Cargo.

`runtime.lock.json` pins every upstream download and every extracted asset by
SHA-256. `provenance.json` records the sources and native ELF checks. Policies come
from Pollen's `microduck-policies` revision
`665ae4eb20bfe13bca1ef4ea6d4bb6b4a096e448`; their model card specifies Apache-2.0.
The pickup detector and Pollen license come from `microduck` revision
`b2b0d213a402caeac492dd662ea891faa17f67c0`.

The policy byte assets preserve the upstream ONNX bytes. `manifest.json.bytes`
preserves the accompanying upstream policy manifest and stages as `manifest.json`.
Their `.bytes` extension makes Unity import them as `TextAsset`, so the controller can stage real ONNX files
for Pollen's native loader even when the application is inside an Android APK.
The original Unity Inference Engine models are archived under `External/LegacyUnityPolicies`; that unused inference package is no longer required.

## Verification and reproduction

From the project root, verify the installed assets without network access:

```sh
python3 Tools/PollenRuntime/prepare.py --verify
```

To reproduce the upstream runtime/model payloads, download exactly the versions
and hashes in the lock file:

```sh
python3 Tools/PollenRuntime/prepare.py
```

Downloads are cached under the ignored `Library/PollenRuntime` directory. A failed
checksum aborts the operation. The script extracts named archive members only,
preserves existing Unity metadata/GUIDs, and selects plugins explicitly for each
platform. It does not install system libraries or change the Unity package manifest.

`libmicroduck_control.so` is built by the separate `Tools/PollenControl` tooling.
After rebuilding both targets, package their outputs with:

```sh
python3 Tools/PollenRuntime/prepare.py --verify \
  --core-linux /path/to/linux/libmicroduck_control.so \
  --core-android /path/to/android/libmicroduck_control.so
```

The packaging tool checks their architecture and public ABI exports, records
their hashes in `control-artifacts.json`, preserves matching build/source provenance
as `control-provenance.json`, and creates platform-specific Unity
plugin metadata. Android artifacts must have 16 KiB aligned ELF load segments and
no undeclared non-platform shared library dependencies.

## ONNX Runtime

Both platforms use **ONNX Runtime 1.23.2 CPU**, compatible with the pinned Pollen
`ort` crate's 1.23 API floor. Linux uses Microsoft's official release archive;
Android uses `jni/arm64-v8a/libonnxruntime.so` from Microsoft's Maven Central AAR.
The Java/JNI wrapper and other architectures are not included. The Android library
depends only on Android's `libc`, `libdl`, `libm`, and `liblog`.

Linux Editor loading uses the packaged library's absolute path. Android loading
uses the native package library name `libonnxruntime.so`; Android's loader resolves
it from the installed APK's native library namespace. These binaries are not
Windows or macOS Editor plugins. The packaged Linux controller requires glibc
2.34 or newer (Ubuntu 22.04 and later meet this requirement); ONNX Runtime also
requires the system C++ runtime with GLIBCXX 3.4.22 or newer.

License texts and ONNX Runtime's third-party notices are included under
`Assets/StreamingAssets/NativeLicenses` and are carried into player builds.
ELF and checksum verification does not establish headset runtime behavior or
performance; that requires an Android build and on-device validation.

## Loader and APK checks

After both control libraries are packaged, run:

```sh
python3 Tools/PollenRuntime/validate.py
```

This audits the six native libraries and plugin platform selectors, loads all
three Linux libraries in a fresh native process, checks their runtime API
versions, and cross-compiles an equivalent ARM64 Android loader with Unity's NDK.
The report is written to `Builds/PollenRuntime/validation.json`. An ARM64
cross-build is recorded separately from Android execution.

For an actual Unity Quest build, also run:

```sh
python3 Tools/PollenRuntime/validate.py --apk /path/to/MicroDuck.apk
```

This requires an ARM64 IL2CPP player, compares the deployed native LOAD images
with the verified source libraries (allowing debug/symbol stripping), verifies
16 KiB ELF and uncompressed ZIP alignment, and runs the Android SDK's zipalign
checker. It does not infer that headset input, policy staging, or physical
recovery works merely from a valid APK.

## Actual physics diagnostic

`physics_trial.py` runs the packaged native core against MuJoCo 3.15.0 and the
project's robot XML with an isolated flat floor. It stages the same byte assets,
uses the same sensor frames and mouth-slot mapping as Unity, settles for ten
simulation seconds, then continuously commands each requested forward speed for
four seconds. This optional developer diagnostic needs Python MuJoCo; the Unity
application does not.

```sh
uv run --no-project --with mujoco==3.15.0 python Tools/PollenRuntime/physics_trial.py \
  --forward 0 .18 .3 .4 .6 .8
```

The full trace and source hashes are written to
`Builds/PollenRuntime/physics-trial.json`. With the packaged controller and current
XML, measured forward travel over four seconds was 0.12 mm at zero command,
8 mm at 0.18 m/s, 334 mm at 0.30 m/s, and 883 mm at 0.60 m/s. This establishes
actual sensor-to-actuator movement, not accurate requested-speed tracking or
closed-loop navigation success. The original desktop scene and this branch's
robot XML are identical.

The existing XML position actuators remain a simplified plant. The velstand
manifest's training commit `3750776` uses BAM M6 motor/friction behavior, voltage
variation, delayed commands and backlash in `microduck_rl`; those dynamics are
not reproduced merely by using the same ONNX network and Pollen controller.
Low-speed movement and physical fall recovery must therefore be evaluated in
the actual scene rather than inferred from controller-equivalence tests.
