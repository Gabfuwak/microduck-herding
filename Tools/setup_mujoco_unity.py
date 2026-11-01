#!/usr/bin/env python3
"""Set up or regenerate the vendored MuJoCo bindings and MicroDuck model.

Ready-to-open project: use Unity 6000.6.3f1 on Linux x86_64. The robot assets,
raw C# bindings and Linux/Android ARM64 native libraries are checked in.
Windows/macOS Editor native libraries are not supplied. Regeneration is optional,
not part of normal setup.

Open Assets/Scenes/MainScene.unity. Desktop controls: mouse look, WASD/arrow
movement, Space/Shift up/down and left-click placement. Desktop scripts and the
AR simulation environment live in Assets/Desktop; robot code/assets live in
Assets/Robots/MicroDuck. For Quest, install Unity's Android Build Support module,
select the Android build profile and build MainScene for ARM64. Use the existing
XR placement gesture: first use places one duck; later uses set its walking
destination. A new destination replaces the previous one. Reset despawns it.
Joystick input cancels navigation: left stick forward/back (up to +/-0.4 m/s) and
sideways (+/-0.3 m/s), right stick horizontal turns (+/-1 rad/s), all duck-relative.
Release to stand. Manual driving requires both controllers tracked and neutral
sticks after spawn/reconnect/focus/pause. Navigation also works with hands only.
The XR Origin Hands rig variant disconnects both controllers' teleport mode/cancel
actions. Right-stick forward no longer starts the template's red invalid-teleport
ray or switches away from normal UI pointing. Robot joystick mappings are unchanged.

Hand UI lives in Assets/UI/MicroDuck and is included in MainScene:
- MicroDuckHandMenu.prefab shares tracked anchors between two stock XRI HandMenu
  followers. Settings appears when the selected palm faces the viewer; the
  microphone appears when the back of that hand faces the viewer. Left is default.
- MicroDuckMicrophone.prefab is a circular, hand-locked mute toggle.
  MicroDuckMicrophoneView swaps its normal/crossed-out icon and tint.
- MicroDuckMainMenu.prefab is the viewer-facing Settings panel beside the palm.
  It uses a standard Reset Button and Left/Right ToggleGroup; MicroDuckSwitchView
  updates the Planes toggle's track colour and thumb position.
Both canvases use TrackedDeviceGraphicRaycaster with the existing XR input rig.
Placement, size, follow behaviour and hit-area padding are configured in the
prefabs. Both view scripts update on value changes and reveal, not every frame.

MicroDuckHandMenuControls lives on the always-active menu root. MainScene assigns
its spawner and ARPlaneManager references; the prefab assigns its controls/followers.
It binds UI and plane-added events, without per-frame searches:
- Reset robot despawns the live duck and returns to the initial placement flow.
  Its simulation resources are released; the existing preload path prepares a new
  hidden duck. The next placement gesture chooses its position. Before placement,
  Reset does nothing.
- Planes changes Renderer.forceRenderingOff on existing and newly detected AR
  planes. Tracking, mesh generation, placement raycasts and MuJoCo collisions stay
  active; it does not hide the physical room or desktop simulation environment.
- Left/Right moves both UI followers to the selected hand, with mirrored offsets.
The microphone remains visual-only: no capture, speech or expansion is connected.
Toggle values survive hiding/revealing the menu instance, not app restarts.

Navigation reuses Andre's planner/follower from commit
b665b088809730077afd4ffc91884cf65df2e15f, without its desktop host or robotd.
MicroDuckNavigator snapshots the same valid box/mesh surfaces as MuJoCo when the
duck is placed, then bakes one NavMesh asynchronously. Place the duck after the
saved room planes have appeared. Walking requests received during the bake wait
for it; head motion, plane visibility and destination changes do not rebuild it.
Reset/replacement takes a fresh snapshot if room geometry was recaptured or moved.
Route queries replan periodically on that fixed NavMesh. The follower corrects
heading/lateral drift from the actual simulated trunk and feet, and returns
robot-relative velocity requests to MicroDuckJoystick, the command owner.
MuJoCo/the existing policies still move and balance the robot, not a NavMeshAgent.
The target circle/route is green for a complete route and amber when no route is
available. Invalid surface points keep the previous goal. Disconnected surfaces
do not become jumps; a walkable but unreachable goal stays pending and replans.
This does not add speech, new policies, stand-up recovery or dynamic room mapping.

User-run headset check:
- Reset before placement, then after walking/falling: the duck should disappear
  and stay absent until a new placement gesture. Place it elsewhere and repeat;
  verify only one duck appears and sticks must return to neutral before driving.
- Hide Planes, scan new surfaces, and verify the duck still collides with hidden
  surfaces. Show Planes again to restore their normal tracking-dependent visibility.
- Switch Left/Right both ways, turning the selected hand to reveal Settings/mic.
  Use the opposite hand for pokes/ray clicks; confirm selections survive hide/reveal.
- Check mic mute feedback and that UI clicks do not place another duck behind it.
- Push the right stick forward and release, including after switching between
  hands and controllers: no teleport ray should appear. Check right-stick turning,
  left-stick movement, controller UI clicks and placement still work.
- After the room planes appear, place the duck and tap/point farther along the
  same surface. Confirm a target circle/route appears and the duck walks toward
  it, including hands-only tracking. Retarget it while walking.
- Try a route around an obstacle, then a disconnected surface/table: the latter
  should have no complete route, not drive directly off an edge toward the goal.
- Move either stick while navigating: the route/circle should clear and manual
  driving should take over. Release to stand, then select another destination.
- Open a system menu while walking: movement requests should stop; returning
  should resume toward the goal. Reset should remove duck, route and target even
  if a bake is still pending; place again to start a fresh navigation instance.
Do not launch Unity builds or automated tests on the user's behalf.

Share source through Git: Assets, Packages, ProjectSettings, Tools and the source
submodule reference. Keep .meta files with their assets. Put player builds under
Builds/; Library, Temp, Logs, UserSettings and Python caches are local/generated.
Local historical captures and builds are archived under Library/HandoffArchive
and are not part of the project handoff.

To regenerate Linux/native model assets, first initialize the source submodule
from the project root:
    git submodule update --init --recursive
Then run with Python 3.10+:
    python3 Tools/setup_mujoco_unity.py
The script uses pinned source and keeps its source/build cache under Library.
To rebuild the Android ARM64 library (Linux host), install Unity's Android Build
Support including Android SDK/NDK and OpenJDK, or provide an Android NDK with
--ndk PATH / ANDROID_NDK_ROOT:
    python3 Tools/build_mujoco_android.py [--ndk PATH]
This rebuild also requires CMake and Git; generated output replaces the checked-in
Android plugin and native license notices. Unity Android player builds additionally
need the Android Build Support module (SDK, NDK and OpenJDK) installed for the
same Unity version. The smoke check requires a connected ARM64 Android device
with USB debugging and adb:
    python3 Tools/smoke_mujoco_android.py [--ndk NDK] [--serial SERIAL]
    python3 Tools/smoke_mujoco_android.py --build-only [--ndk NDK]

In Unity, Tools/MicroDuck/Rebuild MuJoCo Prefab regenerates visuals from the
submodule's full visual MJCF and binary STLs; use it outside Play mode. Configure
Locomotion Prefab refreshes native/policy bindings without regenerating CAD.
Generated visuals have no PhysX colliders or rigidbodies. Python regression
tests for source-model pruning can be run with:
    python3 -m unittest discover -s Tools

Runtime: MicroDuckSpawner prepares one hidden duck before placement, warming the
native world and both CPU policies; placement reuses resources and resets state.
Physics runs at 200 Hz and policy inference at 50 Hz. MicroDuckJoystick owns
manual/navigation velocity selection before each physics update. It reads raw
OpenXR sticks with the official per-axis 0.1 dead zone, without rescaling.
The policy controller smooths commands with alpha 0.2 at 50 Hz and stands at magnitude
<=0.05. SetVelocityCommand must be refreshed continuously: requests expire after
0.5 real seconds. Manual release/tracking loss clears manual input; an active
navigation goal continues without controllers. Focus/pause loss stops requests
without disabling physical balancing and retains the goal for resume. Disabling
the joystick cancels navigation. Placement clears command history; every newly
placed duck requires neutral sticks before manual driving. These are velocity
requests, not guaranteed motion; navigation adds pose feedback for drift correction.
The current checked-in walking policy barely translates under pure lateral requests
in the native smoke run; forward-plus-sideways does move diagonally. Do not treat
the command limits as achieved speeds or guaranteed strafing.
Official command handling: https://github.com/pollen-robotics/microduck/tree/main/pad-map
and https://github.com/pollen-robotics/microduck/blob/main/robotd/src/control.rs
Native body poses drive CAD through ROS-to-Unity (-y,z,x); the spawn anchor must have unit world
scale. MicroDuckSurfaces captures static/kinematic boxes and meshes on AR and
simulation layers 7/30; there is no implicit floor. Shape/add/remove rebuilds
preserve robot integration state and policy history. Dynamic PhysX interaction
and stand-up recovery are not implemented. Opt-in performance capture records
bounded frame/operation percentiles and rebuild reasons.

Visual source: https://github.com/pollen-robotics/microduck_rl
Hardware: https://github.com/pollen-robotics/microduck (Creative Commons BY-SA-NC,
per the source README). Preserve source attribution when redistributing derived
visual meshes. Native MuJoCo license notices remain in StreamingAssets.
"""

from __future__ import annotations

import hashlib
import json
import shutil
import sys
import tarfile
import tempfile
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath

PROJECT = Path(__file__).resolve().parents[1]
PIN = "9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5"
# Official release checksum: releases/download/3.15.0/mujoco-3.15.0-linux-x86_64.tar.gz.sha256.
LINUX_SHA256 = "319943b332fd84c968f655f695d8f4b03cc6a68f658bacd0591f0a552744e497"
LINUX_ARCHIVE = "mujoco-3.15.0-linux-x86_64.tar.gz"
LINUX_URL = f"https://github.com/google-deepmind/mujoco/releases/download/3.15.0/{LINUX_ARCHIVE}"
UNITY_SOURCE = PROJECT / "Library/MuJoCoAndroid/source/unity"
SOURCE_ROOT = PROJECT / "Library/MuJoCoAndroid/source"
ROBOT_DIR = PROJECT / "External/microduck_rl/src/mjlab_microduck/robot/microduck"
PACKAGE_OUT = PROJECT / "Packages/org.mujoco"
PLUGIN_OUT = PROJECT / "Assets/Plugins/Linux/x86_64/libmujoco.so"
PLUGIN_META = Path(str(PLUGIN_OUT) + ".meta")
PROVENANCE = PROJECT / "Library/MuJoCoUnity/setup-report.json"
MODEL_OUT = PROJECT / "Assets/Robots/MicroDuck/MuJoCo"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def copy_package(commit: str) -> None:
    if not (UNITY_SOURCE / "package.json").is_file() or not (SOURCE_ROOT / "LICENSE").is_file():
        raise RuntimeError(f"Pinned checkout is missing official Unity package or LICENSE: {SOURCE_ROOT}")

    retained = (
        "package.json", "package.json.meta", "README.md", "README.md.meta",
        "CHANGELOG.md", "CHANGELOG.md.meta", "Runtime.meta",
        "Runtime/Mujoco.Runtime.asmdef", "Runtime/Mujoco.Runtime.asmdef.meta",
        "Runtime/Bindings.meta", "Runtime/Bindings/MjBindings.cs",
        "Runtime/Bindings/MjBindings.cs.meta",
    )
    for relative in retained:
        if not (UNITY_SOURCE / relative).is_file():
            raise RuntimeError(f"Pinned checkout is missing required binding file: {relative}")
    # Remove obsolete integration, but preserve retained Unity metadata and GUIDs.
    allowed = set(retained) | {"LICENSE", "LICENSE.meta"}
    if PACKAGE_OUT.exists():
        for path in sorted(PACKAGE_OUT.rglob("*"), key=lambda path: len(path.parts), reverse=True):
            if path.is_dir():
                if not any(path.iterdir()):
                    path.rmdir()
            elif path.relative_to(PACKAGE_OUT).as_posix() not in allowed:
                path.unlink()
    for relative in retained:
        destination = PACKAGE_OUT / relative
        if relative.endswith(".meta") and destination.exists():
            continue
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(UNITY_SOURCE / relative, destination)
    (PACKAGE_OUT / "README.md").write_text(
        "# MuJoCo raw Unity bindings\n\n"
        "This package contains the official MuJoCo 3.15.0 C# bindings "
        "(`MujocoLib`) and their runtime assembly only. It does not include "
        "Unity components, MJCF import/export tooling, or Editor integration. "
        "Install the matching MuJoCo native library separately.\n",
        encoding="utf-8",
    )
    (PACKAGE_OUT / "CHANGELOG.md").write_text(
        "# MuJoCo raw Unity bindings\n\n"
        "## 3.15.0\n\n"
        "Vendored the official MuJoCo 3.15.0 `MujocoLib` raw bindings and "
        "runtime assembly only; Unity component and Editor integration is "
        "excluded.\n",
        encoding="utf-8",
    )
    metadata_path = PACKAGE_OUT / "package.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata["description"] = "Raw MuJoCo C bindings for Unity"
    metadata["mujocoSource"] = {
        "repository": "https://github.com/google-deepmind/mujoco.git",
        "tag": "3.15.0",
        "commit": commit,
        "unityPackage": "unity/",
        "localChanges": ["raw bindings only; omit runtime components, editor integration, tests, and automatic binary retriever"],
    }
    metadata_path.write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    shutil.copy2(SOURCE_ROOT / "LICENSE", PACKAGE_OUT / "LICENSE")


def install_linux_plugin() -> str:
    with tempfile.TemporaryDirectory(prefix="mujoco-3.15.0-") as temp_name:
        temp = Path(temp_name)
        archive = temp / LINUX_ARCHIVE
        request = urllib.request.Request(LINUX_URL, headers={"User-Agent": "MuJoCo-Unity-project-setup"})
        with urllib.request.urlopen(request, timeout=120) as response, archive.open("wb") as output:
            shutil.copyfileobj(response, output)
        digest = sha256(archive)
        if digest != LINUX_SHA256:
            raise RuntimeError(f"Official Linux release archive SHA-256 mismatch: expected {LINUX_SHA256}, got {digest}")
        with tarfile.open(archive, "r:gz") as bundle:
            expected = "mujoco-3.15.0/lib/libmujoco.so.3.15.0"
            member = next((item for item in bundle.getmembers() if item.name == expected), None)
            if member is None or not member.isfile():
                raise RuntimeError(f"Release archive lacks expected regular file {expected}")
            source = bundle.extractfile(member)
            if source is None:
                raise RuntimeError(f"Cannot extract {expected}")
            PLUGIN_OUT.parent.mkdir(parents=True, exist_ok=True)
            binary = source.read()
            source.close()
            if PLUGIN_OUT.exists() and PLUGIN_OUT.read_bytes() != binary:
                raise RuntimeError(f"Existing Linux plugin differs from the verified release: {PLUGIN_OUT}")
            if not PLUGIN_OUT.exists():
                PLUGIN_OUT.write_bytes(binary)
    PLUGIN_OUT.chmod(0o755)
    plugin_hash = sha256(PLUGIN_OUT)
    if not PLUGIN_META.exists():
        PLUGIN_META.write_text("""fileFormatVersion: 2
guid: 2cbba7c554744e15aa41fd040e3b7a59
PluginImporter:
  externalObjects: {}
  serializedVersion: 2
  iconMap: {}
  executionOrder: {}
  defineConstraints: []
  isPreloaded: 0
  isOverridable: 0
  isExplicitlyReferenced: 0
  validateReferences: 1
  platformData:
  - first:
      Any:
    second:
      enabled: 0
      settings: {}
  - first:
      Editor: Editor
    second:
      enabled: 1
      settings:
        CPU: x86_64
        OS: Linux
        DefaultValueInitialized: true
  - first:
      Standalone: Linux64
    second:
      enabled: 1
      settings:
        CPU: x86_64
  userData:
  assetBundleName:
  assetBundleVariant:
""", encoding="utf-8")
    PROVENANCE.parent.mkdir(parents=True, exist_ok=True)
    PROVENANCE.write_text(json.dumps({
        "source": "https://github.com/google-deepmind/mujoco/releases/tag/3.15.0",
        "asset": LINUX_ARCHIVE,
        "assetUrl": LINUX_URL,
        "releaseAssetSha256": LINUX_SHA256,
        "archiveSha256": digest,
        "member": expected,
        "installedFileSha256": plugin_hash,
        "architecture": "Linux x86_64",
    }, indent=2) + "\n", encoding="utf-8")
    return plugin_hash


def _remove_visual_geometry(robot: ET.Element) -> None:
    # Only the upstream visual class is eligible. Respect per-geom collision
    # overrides and explicit geom references even when automatic masks are zero.
    visual = robot.find(".//default[@class='visual']/geom")
    visual_defaults = visual.attrib if visual is not None else {}
    referenced = {
        element.get(attribute)
        for element in robot.iter()
        for attribute in ("geom", "geom1", "geom2")
        if element.get(attribute)
    }
    referenced.update(
        element.get("objname") for element in robot.iter()
        if element.get("objtype") == "geom" and element.get("objname")
    )
    for world in robot.findall("worldbody"):
        for parent in world.iter():
            for geom in list(parent.findall("geom")):
                if geom.get("class") != "visual" or geom.get("name") in referenced:
                    continue
                contype = int(geom.get("contype", visual_defaults.get("contype", "1")))
                conaffinity = int(geom.get("conaffinity", visual_defaults.get("conaffinity", "1")))
                if contype == 0 and conaffinity == 0:
                    parent.remove(geom)


def make_model() -> None:
    source_xml = ROBOT_DIR / "robot_allcollisions.xml"
    scene_xml = ROBOT_DIR / "scene.xml"
    if not source_xml.is_file() or not scene_xml.is_file():
        raise RuntimeError(f"Missing pinned MicroDuck source MJCF under {ROBOT_DIR}")
    robot = ET.parse(source_xml).getroot()
    scene = ET.parse(scene_xml).getroot()
    # Keep the full-contact robot, but Unity supplies its CAD rendering.
    _remove_visual_geometry(robot)
    for child in list(scene):
        if child.tag == "include":
            scene.remove(child)
        elif child.tag == "asset":
            scene.remove(child)
    root_body = next((body for body in robot.iter("body") if body.get("name") == "trunk_base"), None)
    if root_body is None:
        raise RuntimeError("Source model lacks trunk_base")
    root_body.set("pos", "0 0 0.119182773")
    for keyframes in list(robot.findall("keyframe")):
        robot.remove(keyframes)
    option = robot.find("option")
    if option is None:
        option = ET.Element("option")
        robot.insert(0, option)
    option.set("timestep", "0.005")
    # Remove source keyframes while retaining defaults, bodies, inertias,
    # contact, actuators, sensors, equality and every collision geom.
    scene_visual = scene.find("visual")
    if scene_visual is not None:
        robot.append(scene_visual)
    world = robot.find("worldbody")
    if world is None:
        raise RuntimeError("Source model lacks worldbody")
    scene_world = scene.find("worldbody")
    if scene_world is not None:
        for element in list(scene_world):
            # Unity supplies bounded table/floor geometry at runtime; an infinite
            # training plane would incorrectly support the duck past every edge.
            if element.tag == "geom" and element.get("name") == "floor":
                continue
            world.append(element)
    referenced_mesh_names = {geom.get("mesh") for geom in robot.iter("geom") if geom.get("mesh")}
    referenced_materials = {element.get("material") for element in robot.iter() if element.get("material")}
    for assets in robot.findall("asset"):
        for mesh in list(assets.findall("mesh")):
            name = mesh.get("name") or Path(mesh.get("file", "")).stem
            if name not in referenced_mesh_names:
                assets.remove(mesh)
        for material in list(assets.findall("material")):
            if material.get("name") not in referenced_materials:
                assets.remove(material)
    # Keep only referenced meshes in XML and VFS. Compiler meshdir remains
    # "assets", so VFS filenames match assets/*.stl.
    mesh_files = sorted({
        mesh.get("file")
        for mesh in robot.findall("./asset/mesh")
        if mesh.get("file") and (mesh.get("name") or Path(mesh.get("file", "")).stem) in referenced_mesh_names
    })
    output_asset_dir = MODEL_OUT / "assets"
    output_asset_dir.mkdir(parents=True, exist_ok=True)
    mesh_copies = []
    for filename in mesh_files:
        relative = PurePosixPath(filename)
        if relative.is_absolute() or ".." in relative.parts or len(relative.parts) != 1:
            raise RuntimeError(f"Unexpected source mesh path: {filename}")
        source_mesh = ROBOT_DIR / "assets" / relative.name
        if not source_mesh.is_file():
            raise RuntimeError(f"Referenced mesh missing: {source_mesh}")
        mesh_copies.append((source_mesh, output_asset_dir / f"{relative.name}.bytes"))
    # Validate all inputs before changing any previously generated asset.
    for source_mesh, destination in mesh_copies:
        shutil.copyfile(source_mesh, destination)
    expected = {destination.name for _, destination in mesh_copies}
    for path in output_asset_dir.iterdir():
        if path.is_file() and (path.name.endswith(".bytes") or path.name.endswith(".bytes.meta")):
            if path.name.removesuffix(".meta") not in expected:
                path.unlink()
    ET.indent(robot, space="  ")
    ET.ElementTree(robot).write(MODEL_OUT / "scene.xml", encoding="utf-8", xml_declaration=True)


def main() -> None:
    # Import the existing helper rather than duplicate or weaken its pin logic.
    sys.path.insert(0, str(PROJECT / "Tools"))
    import build_mujoco_android
    commit = build_mujoco_android.ensure_source()
    if commit != build_mujoco_android.PIN or commit != PIN:
        raise RuntimeError(f"Unexpected source revision: {commit}")
    copy_package(commit)
    plugin_hash = install_linux_plugin()
    make_model()
    print(f"Vendored org.mujoco from MuJoCo {build_mujoco_android.TAG} ({commit})")
    print(f"Linux plugin: {PLUGIN_OUT.relative_to(PROJECT)} (sha256 {plugin_hash})")
    print(f"VFS model: {(MODEL_OUT / 'scene.xml').relative_to(PROJECT)}; meshes: {len(list((MODEL_OUT / 'assets').glob('*.bytes')))}")


if __name__ == "__main__":
    main()
