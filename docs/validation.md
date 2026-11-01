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
| Behind-target command bounds | Yaw ≤0.55 rad/s, slew ≤0.016 per 20 ms; stable direction around 180° |
| Full physical behind-target trial at 4× | Arrived upright after 8.84 simulation seconds, approximately 165° measured turn, then reached a second goal |
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
trial; its menu check performs the table/drop trial.

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
