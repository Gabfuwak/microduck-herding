# MicroDuck in native MuJoCo

The official Google DeepMind Unity plugin is embedded at `Packages/org.mujoco`,
pinned to MuJoCo 3.15.0 (git commit 9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5).
The matching universal macOS library is bundled in `Plugins/macOS`.
The package retains its Apache-2.0 license. Local compatibility changes replace
GetInstanceID with Unity 6.6's GetEntityId and recognize the bundled library.

The controller uses the plugin's MujocoLib bindings and loads MJCF directly,
rather than rebuilding the dynamics from Unity components. Each duck owns its
own model/data and disposes both on disable. Physics steps at 0.005 s; ONNX
inference runs every four steps. Unity's global timestep need not change.
Targets are default pose + raw action radians; no extra normalization is applied.
The walking policy also handles zero velocity; the standing policy is selected
for fall recovery. The existing URDF meshes are visual only, with all PhysX
colliders and ArticulationBodies removed from the new prefab.

## Use

- With MainScene open, Tools > MicroDuck > Set Up MuJoCo creates the new prefab and sandbox and
  replaces the old robot choice in MainScene's ObjectSpawner.
- Tools > MicroDuck > Open MuJoCo Sandbox, then Play.
- Automatic walking begins after five simulation seconds, following a circle.
- Arrow keys steer while held; Space stands while held; R resets.
- Other scripts can call SetVelocityCommand(forward, left, yawRate), using
  robot-frame metres/second and radians/second. This disables automatic commands.
- MainScene retains its desktop/XR placement flow; spawned ducks use MuJoCo.

## Current scope

The macOS editor is verified. An Android ARM64 MuJoCo 3.15.0 library is now
bundled in Plugins/Android/arm64-v8a, built from upstream source. The Android
IL2CPP smoke APK builds successfully and extracts packaged assets into app
storage. A native loader/model-stepping probe passed on Quest 3S. The full
Unity/ONNX headset run remains pending because the headset was disconnected
before APK installation. See Tools/quest_smoke_test.md for evidence and commands.
Windows and Linux still require matching native binaries and validation.

Each duck simulates on a horizontal plane in its own spawn frame. Consequently,
ducks do not collide with each other, Unity props, hands, or scanned room geometry.
They also do not fall off a real table edge. A shared MuJoCo world and explicit
collision geometry for the room are the next integration step for physical
herding. The demo circle reduces wandering but is not edge avoidance.
Do not scale the prefab: rendering scale would differ from physical dimensions.

## Sources and attribution

Model and policies copied from pollen-robotics/microduck-simulator, revision
023172c, paths app/public/robot/mjlab/robot_allcollisions.xml and
app/public/policies/BEST_alpha_{walking,stand}.onnx.
https://huggingface.co/spaces/pollen-robotics/microduck-simulator/tree/023172c

The MJCF keeps inertials, joint damping/friction/armature, actuator gains and
limits, contact masks and IMU sensors. Visual geoms and unused visual mesh
assets are stripped; a plane and a 0.005 s timestep are added, following the
Space's preparation. Collision meshes reuse the project's existing source STL
assets. Original MJCF is retained beside the prepared model as source.xml.

Hardware design attribution: MicroDuck, https://github.com/pollen-robotics/microduck
and https://github.com/pollen-robotics/microduck_rl. The upstream hardware design
notice is Creative Commons BY-SA-NC; copied/derived design assets retain that
notice. Original Onshape design:
https://cad.onshape.com/documents/804927696f06d877f3f1803e/w/5b75db19292e71970de02dee/e/ef6e972847fec8d82570b35e
Policy files are published by Pollen Robotics. See the source repository/model
license notices; the project's older policy provenance remains documented in
Tools/convert_microduck.py.

## Validation (2026-10-07)

Unity 6000.6.3f1 on macOS: sandbox walking and turning stayed upright for
50 simulated seconds with zero fall-recovery events. Game-view R reset returned
to the initial pose and restarted the clock. The 30-second native Python/MuJoCo
reference rollout also stayed upright (minimum trunk height 0.1130 m).
Unity Inference Engine action outputs matched ONNX Runtime on a recorded Unity
walking observation to maximum absolute error 1.8e-7.
Desktop MainScene placement was also verified through the Unity UI: the new
prefab spawned on the table and walked/turned for 120 simulated seconds with
zero fall-recovery events.

The demo requests 0.3 m/s forward and 0.4 rad/s yaw. With the current source
model/position actuators, 0.2 m/s did not initiate a gait in either Unity or the
reference rollout; avoid assuming arbitrarily slow commands will produce motion.
This follows the browser simulator's position-actuator setup. The main MicroDuck
repository's duck-sim uses BAM motor models and the real daemon control loop;
those additional motor/runtime dynamics are not part of this integration.
Reference: https://github.com/pollen-robotics/microduck/blob/main/docs/robot/simulation.md
