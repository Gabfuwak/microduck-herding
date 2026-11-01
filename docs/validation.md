# Integration validation

Validated on Ubuntu 25.10, Unity 6000.6.3f1, MuJoCo 3.15.0 and ONNX Runtime
1.23.2 CPU. The integration starts from `origin/wip/quest-navigation` at
`a5f088e77c92f9abbae9f1f4e095e7a00d650647`; it does not merge main or the older
daemon-based feature branch.

| Check | Result |
| --- | --- |
| Pollen upstream unit/inference tests | 89 passed, none ignored |
| Native core vs actual robotd, 300 scripted sensor ticks | All 15 targets, gains and torque identical; maximum target error 0 |
| Native reset/pause replay; daemon 1×/4× | Identical outputs |
| Real Unity/MuJoCo startup, walking and forced supine recovery | Passed; 0.32 m forward over 4 simulation seconds at a 0.3 m/s request; upright afterward |
| Simulation time through that recovery | Every physics interval retained; no clock restart |
| Generated table-to-floor route | Complete; reverse climb blocked |
| Behind-target command bounds | Yaw ≤0.55 rad/s, slew ≤0.016 per 20 ms; stable direction around 180°; nonzero turn at zero requested walking speed |
| Full physical behind-target trial at 4× | After separating route heading from walking speed: arrived upright after 8.47 simulation seconds, approximately 161° measured turn, then reached a second goal |
| Actual “Goal behind duck” button, 0.65 m target | Reached the accepted target upright in the editor trial |
| Physical 0.45 m table drop at 4× | Fell, retained goal, landed, resumed, arrived upright; placement anchor unchanged |
| Invalid target while navigating | Previous goal retained |
| Linux standalone with fresh XDG policy cache | Built and ran; policies staged from packaged assets, no native/XR startup errors |
| Android ARM64 IL2CPP app | Built successfully; native payloads match packaged loadable images |
| Android native libraries and APK alignment | Architecture, dependencies, 16 KiB ELF alignment and SDK zipalign passed |
| Clean copy with no Library cache | Imported 3,998 assets, compiled and passed native walking/recovery checks; Unity exit 0 |
| Quest device execution | Pending; no ARM64 Quest is connected locally |

Machine-generated details are under ignored `Builds/Validation`,
`Builds/PollenControl`, and `Builds/PollenRuntime`. Reproduction instructions are
in the two native tooling READMEs. `MicroDuckControlChecks` uses a deliberately
forced supine pose as a test input; production resets occur only on explicit
placement. `MicroDuckNavigationChecks.RunFlat()` performs the full behind-target
trial; its menu check performs the table/drop trial. Steering heading comes
from `NavMeshAgent.steeringTarget` relative to measured feet, independently of
`desiredVelocity`, so a zero requested walking speed does not suppress turning.
Translation, turn limits and acceleration limits are unchanged.

## Refactor check (2026-11-01)

Compared with the working `9060321` baseline, the refactor names the existing
native ABI states and shares repeated scan/physics setup, recovery/progress
handling and mesh serialization. Policy bytes, native binaries, tuning, prefab,
Quest scene and input bindings are unchanged.

| Check | Result |
| --- | --- |
| Geometry before/after refactor | Identical XML bytes, navigation sources, bounds and refresh flags across initial, unchanged, moved, reshaped and removed surfaces; rotated/nonuniform boxes and triangle meshes included |
| Native startup, walking and forced fall | Passed in the open editor and isolated copy; same 0.32003 m forward travel and continuous clock |
| Physical behind-goal and next goal | Isolated trial arrived after 8.32 simulation seconds and a 165 degree turn; open-editor repeat also passed |
| Table drop with recovery | Open-editor trial fell, retained its goal, recovered and arrived upright; isolated drop arrived without requiring recovery |
| Live room geometry changes | Pose update and size/add/remove recompiles preserved robot state bit-for-bit, controller instance, clock, gravity and timestep; next step remained finite and advanced 20 ms |
| Quest profile from Linux Editor | Selected and switched through Unity's actual Build Profiles window; Android target/group, MainScene, IL2CPP and ARM64 confirmed |

The Quest profile also produced a non-development ARM64 IL2CPP APK with zero
build errors. Native payload and 16 KiB ELF/ZIP alignment checks passed; source
and XR settings matched the build copy. Reports are alongside the ignored APK
under `Builds/Android/refactor-*`. Device execution remains untested.

One earlier open-editor behind-goal trial exceeded its 30 simulation-second
bound; its later forward goal still arrived. The repeat and isolated trial passed,
so these results establish exercised cases rather than deterministic arrival.
Reports are under `Builds/Validation/refactor-*` and `geometry-equivalence.json`.

## Practical limits

Native trace equality covers the enabled navigation configuration over the
scripted observations. It does not establish equivalence to physical servos or
every daemon feature. The native ABI exposes continuous velocity and automatic
recovery; explicit kick/sit/ground-pick requests, device buses and daemon IPC are
outside this adapter.

The inherited MuJoCo scene is unchanged. It uses position actuators and a motor
load proxy. The exact velstand training revision uses BAM motor dynamics, voltage,
delay and backlash; those are not all reproduced by this scene. A separate real
MuJoCo speed sweep confirmed weak motion at small velocity requests and sideways
drift. Navigation therefore uses the existing manual-control range (0.4 m/s),
Unity's clearance/steering, measured feedback and physical arrival, with advisory
braking disabled. No policy gains or trained action filters were retuned.

Retaining a destination does not guarantee escape from geometry that traps the
robot. The successful table trial is one concrete recovery case. Real Quest
input, application lifecycle, CPU budget and sustained on-device performance
still require headset validation.
