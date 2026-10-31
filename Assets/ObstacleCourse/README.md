# MicroDuck navigation trial

First run `python3 Tools/Robotd/prepare.py` on Linux x86_64. Then open
`Assets/Scenes/MicroDuckObstacles.unity` and press Play. Simulation runs at
4x speed; physics remains 200 Hz and policy control 50 Hz in simulation time.

- Left-click a walkable surface to select or replace the destination.
- The green line is the NavMesh route; the ring marks the destination.
- Space cancels. WASD / QE switch to manual control and cancel navigation.
- Right-drag orbits the camera; scroll zooms.
- Losing focus pauses navigation and retains the destination.
- Falls never reset or respawn the duck. The HUD shows the motor/policy phase.

`MicroDuckNavigator` builds a runtime Unity NavMesh from the scene's layer-7 box
colliders. It sends robot-relative velocity requests to the policy; MuJoCo alone
moves the duck. Steering uses the simulated trunk pose, and path sampling uses
the measured foot midpoint. The root Transform is only the simulation anchor.

The follower aims 14 cm ahead along the route, corrects forward/lateral/yaw
requests every fixed tick and replans every 0.6 simulated seconds. Arrival uses
a 5.5 cm horizontal tolerance plus compatible surface height. A missing route
retains the goal and retries. Ordinary stalls trigger bounded movement retries.

Fall handling follows Pollen Robotics' finite sequence: predict a new fall, soften
the motors while following measured joints, wait for landing, ramp motor targets
from landed joint angles to home, then return control to the actor. Hand-back
is time bounded and never waits indefinitely for upright. A fallen duck remains
policy-driven; the finite limp/pose sequence pauses route following, followed by at most
three simulated seconds at zero velocity for getting up. Movement retries resume
after that window if the robot remains trapped.
Physical body state is preserved throughout, with no reset shortcut.

The scene runs Pollen's native Rust `robotd` with the pinned official `velstand.onnx`.
Unity supplies sensor readings and applies motor commands; robotd owns inference,
safety and recovery. An opt-in clock patch synchronizes 50 Hz control with 200 Hz
MuJoCo physics at 4x speed. The simulator disables pickup detection because its
motor-current features are not faithfully modeled and caused a false pause on a box.
See `Tools/Robotd/README.md` for the exact boundary, setup, pins and validation.

Navigation still assumes static course geometry. The 8.5 cm NavMesh step allowance
is a planning parameter, not a guarantee of physical traversal. A difficult
contact can still trap the robot; retaining a goal is not a guarantee of escape.

Historical validation of the earlier C# reference controller (not robotd): a route from the starting box to
(1.2, 0, 1.1) produced two step-edge falls; both completed limp/pose hand-back,
resumed movement and reached the target. A second goal to (-0.9, 0, 1.6) retained
its destination through an externally applied physical topple and also arrived.
The native simulation clock remained continuous. Local telemetry is saved under
`Builds/Validation/pollen-recovery*`. Pure control tests cover 13 regression cases,
including no perpetual upright gate and no repeated limp loop during a get-up.

Current robotd validation is saved under `Builds/Robotd/validation/`. The scripted
clock test compares all motor outputs at 1x/4x and verifies pause and transport
failure semantics. Physical trials exercised walking and recovery, and also found
poses near steps where the policy keeps trying without getting free.
