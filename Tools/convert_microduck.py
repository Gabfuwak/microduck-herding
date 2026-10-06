#!/usr/bin/env python3
"""Convert the pinned MicroDuck full-collision MJCF into a Unity import URDF.

Usage (from the Unity project root): python3 Tools/convert_microduck.py
Then run Tools/MicroDuck/Rebuild Physics Prefab in Unity, outside Play mode.
The same prefab spawns from desktop or XR: gravity, limited hinges, convex
contacts and no motor drives. This is a passive PhysX ragdoll, not locomotion.
Collision meshes are pre-baked during player builds; editor script reloads
prepare the collision cache so spawning does not cook the CAD meshes.

Source model: https://github.com/pollen-robotics/microduck_rl,
  src/mjlab_microduck/robot/microduck/robot_allcollisions.xml
Hardware-design attribution: MicroDuck, https://github.com/pollen-robotics/microduck
The source README licenses hardware design files under Creative Commons
BY-SA-NC; this derived model and its copied meshes retain that attribution and
license notice. Original Onshape design: https://cad.onshape.com/documents/804927696f06d877f3f1803e/w/5b75db19292e71970de02dee/e/ef6e972847fec8d82570b35e

This scoped converter preserves MJCF SI transforms/inertials and mesh geoms.
Collision classes become ordinary URDF collisions; MuJoCo collision masks and
actuator dynamics are not reproduced. Sensors, sites, cameras and world placement
are omitted.
URDF-required velocity 1 rad/s is a non-source fallback; the Unity ragdoll uses
a 50 rad/s angular speed cap instead. Effort is chosen_actuator's ±0.96
forcerange, with drives disabled in Unity. Mesh scale must be omitted (default 1).
"""

from __future__ import annotations

import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
SOURCE = PROJECT / "External/microduck_rl/src/mjlab_microduck/robot/microduck/robot_allcollisions.xml"
ASSET_DIR = SOURCE.parent / "assets"
OUTPUT = PROJECT / "Assets/Robots/MicroDuck/microduck.urdf"
MESH_OUTPUT = OUTPUT.parent / "Meshes"
VELOCITY_FALLBACK = "1.0"
EFFORT = "0.96"


def numbers(value: str, count: int, label: str) -> tuple[float, ...]:
    try:
        result = tuple(float(part) for part in value.split())
    except ValueError as exc:
        raise ValueError(f"Invalid numeric {label}: {value!r}") from exc
    if len(result) != count or not all(math.isfinite(x) for x in result):
        raise ValueError(f"Expected {count} finite values for {label}, got {value!r}")
    return result


def fmt(value: float) -> str:
    if abs(value) < 5e-16:
        value = 0.0
    return f"{value:.12g}"


def vec(values: tuple[float, ...]) -> str:
    return " ".join(fmt(v) for v in values)


def origin(pos: tuple[float, ...], quat: tuple[float, ...]) -> str:
    # MJCF wxyz -> normalized quaternion -> URDF fixed-axis roll, pitch, yaw.
    w, x, y, z = quat
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-15:
        raise ValueError("Zero quaternion")
    w, x, y, z = (v / norm for v in (w, x, y, z))
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1 - 1e-12:
        # At gimbal lock roll/yaw are coupled. Choosing roll=0 avoids atan2(0,0)
        # producing a pair that does not represent the original rotation.
        roll = 0.0
        pitch = math.copysign(math.pi / 2, sinp)
        yaw = math.atan2(2 * (w * z - x * y), 1 - 2 * (x * x + z * z))
    else:
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(sinp)
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return f'<origin xyz="{vec(pos)}" rpy="{vec((roll, pitch, yaw))}"/>'


def attrs(element: ET.Element, key: str, count: int, default: str, label: str) -> tuple[float, ...]:
    return numbers(element.get(key, default), count, label)


def xml_text(value: str) -> str:
    return value.replace("&", "&amp;").replace('"', "&quot;").replace("<", "&lt;").replace(">", "&gt;")


def main() -> None:
    tree = ET.parse(SOURCE)
    root = tree.getroot()
    if root.tag != "mujoco" or root.get("model") != "microduck":
        raise ValueError("Unexpected MJCF root/model")
    compiler = root.find("compiler")
    if compiler is None or compiler.get("angle") != "radian" or compiler.get("meshdir") != "assets":
        raise ValueError("Expected source compiler angle='radian' meshdir='assets'")
    worldbody = root.find("worldbody")
    if worldbody is None or len(worldbody.findall("body")) != 1:
        raise ValueError("Expected exactly one top-level body")
    root_body = worldbody.find("body")
    assert root_body is not None
    if root_body.get("name") != "trunk_base":
        raise ValueError("Expected trunk_base as the root link")
    mesh_assets: dict[str, str] = {}
    asset = root.find("asset")
    if asset is None:
        raise ValueError("Missing asset section")
    for mesh in asset.findall("mesh"):
        if set(mesh.attrib) != {"file"}:
            raise ValueError(f"Unexpected mesh asset settings: {mesh.attrib}")
        filename = mesh.get("file", "")
        key = Path(filename).stem
        if not filename.endswith(".stl") or key in mesh_assets:
            raise ValueError(f"Unsupported or duplicate mesh asset: {filename}")
        mesh_assets[key] = filename
    materials: dict[str, tuple[float, ...]] = {}
    for material in asset.findall("material"):
        name = material.get("name")
        if not name or set(material.attrib) != {"name", "rgba"}:
            raise ValueError(f"Unexpected material settings: {material.attrib}")
        materials[name] = numbers(material.attrib["rgba"], 4, f"material {name} rgba")

    links: list[str] = []
    joints: list[str] = []
    copied: set[str] = set()
    visual_count = 0

    def convert_body(body: ET.Element, parent: str | None = None) -> None:
        nonlocal visual_count
        name = body.get("name")
        if not name:
            raise ValueError("Unnamed body")
        if set(body.attrib) - {"name", "pos", "quat", "childclass"}:
            raise ValueError(f"Unexpected body attributes on {name}: {body.attrib}")
        pos = attrs(body, "pos", 3, "0 0 0", f"{name} pos")
        quat = attrs(body, "quat", 4, "1 0 0 0", f"{name} quat")
        inertial = body.find("inertial")
        if inertial is None or set(inertial.attrib) != {"pos", "mass", "fullinertia"}:
            raise ValueError(f"{name} needs explicit pos, mass, and fullinertia")
        ipos = numbers(inertial.attrib["pos"], 3, f"{name} inertial pos")
        mass = float(inertial.attrib["mass"])
        if not math.isfinite(mass) or mass <= 0:
            raise ValueError(f"Invalid mass on {name}")
        ixx, iyy, izz, ixy, ixz, iyz = numbers(inertial.attrib["fullinertia"], 6, f"{name} inertia")
        links.append(f'  <link name="{xml_text(name)}">')
        links.append(f'    <inertial>{origin(ipos, (1, 0, 0, 0))}<mass value="{fmt(mass)}"/><inertia ixx="{fmt(ixx)}" ixy="{fmt(ixy)}" ixz="{fmt(ixz)}" iyy="{fmt(iyy)}" iyz="{fmt(iyz)}" izz="{fmt(izz)}"/></inertial>')

        for geom in body.findall("geom"):
            if geom.get("type") != "mesh" or geom.get("class") not in {"visual", "collision", "self_collision_only", "collision_only"}:
                raise ValueError(f"Unsupported geom on {name}: {geom.attrib}")
            if set(geom.attrib) - {"type", "class", "pos", "quat", "mesh", "material", "name"}:
                raise ValueError(f"Unexpected geom settings on {name}: {geom.attrib}")
            mesh_name = geom.get("mesh")
            if mesh_name not in mesh_assets:
                raise ValueError(f"Unknown mesh {mesh_name!r} on {name}")
            material = geom.get("material")
            if material not in materials:
                raise ValueError(f"Unknown or missing material {material!r} on {name}")
            filename = mesh_assets[mesh_name]
            source_mesh = ASSET_DIR / filename
            if not source_mesh.is_file():
                raise FileNotFoundError(source_mesh)
            copied.add(filename)
            kind = "visual" if geom.get("class") == "visual" else "collision"
            mesh_uri = f"Meshes/{filename}"
            geom_origin = origin(attrs(geom, "pos", 3, "0 0 0", f"{name} geom pos"), attrs(geom, "quat", 4, "1 0 0 0", f"{name} geom quat"))
            links.append(f'    <{kind}><geometry><mesh filename="{mesh_uri}"/></geometry>{geom_origin}' + (f'<material name="{xml_text(material)}"><color rgba="{vec(materials[material])}"/></material>' if kind == "visual" else "") + f'</{kind}>')
            if kind == "visual":
                visual_count += 1
        links.append("  </link>")

        child_bodies = body.findall("body")
        direct_joints = body.findall("joint")
        if parent is not None:
            if len(direct_joints) != 1:
                raise ValueError(f"Expected exactly one joint in body {name}, got {len(direct_joints)}")
            joint = direct_joints[0]
            if joint.get("type") != "hinge" or joint.get("class") != "chosen_actuator":
                raise ValueError(f"Unsupported joint in {name}: {joint.attrib}")
            if set(joint.attrib) != {"axis", "name", "type", "range", "class"}:
                raise ValueError(f"Unexpected joint settings in {name}: {joint.attrib}")
            jpos = attrs(joint, "pos", 3, "0 0 0", f"{joint.get('name')} pos")
            if any(abs(v) > 1e-12 for v in jpos):
                raise ValueError(f"Nonzero joint position on {joint.get('name')}")
            axis = numbers(joint.attrib["axis"], 3, f"{joint.get('name')} axis")
            low, high = numbers(joint.attrib["range"], 2, f"{joint.get('name')} range")
            if not joint.get("name") or low >= high:
                raise ValueError(f"Invalid joint name/range in {name}")
            joints.append(f'  <joint name="{xml_text(joint.attrib["name"])}" type="revolute"><parent link="{xml_text(parent)}"/><child link="{xml_text(name)}"/>{origin(pos, quat)}<axis xyz="{vec(axis)}"/><limit lower="{fmt(low)}" upper="{fmt(high)}" effort="{EFFORT}" velocity="{VELOCITY_FALLBACK}"/></joint>')
        elif direct_joints:
            if len(direct_joints) != 1 or direct_joints[0].get("type") != "free":
                raise ValueError("Expected only trunk_base_freejoint at root")
            # Omit MJCF world placement and freejoint: trunk_base is URDF root.
        elif name != "trunk_base":
            raise ValueError(f"Missing parent joint for {name}")

        freejoints = body.findall("freejoint")
        if name == "trunk_base":
            if len(freejoints) != 1 or set(freejoints[0].attrib) != {"name"} or freejoints[0].get("name") != "trunk_base_freejoint":
                raise ValueError("Expected trunk_base_freejoint on root")
        elif freejoints:
            raise ValueError(f"Unexpected freejoint in {name}")

        allowed = {"inertial", "geom", "joint", "freejoint", "body", "site", "camera"}
        unexpected = [child.tag for child in body if child.tag not in allowed]
        if unexpected:
            raise ValueError(f"Unsupported elements in body {name}: {unexpected}")
        for child in child_bodies:
            convert_body(child, name)

    convert_body(root_body)
    output_xml = [
        '<?xml version="1.0"?>',
        '<!-- Derived from MicroDuck hardware design, https://github.com/pollen-robotics/microduck -->',
        '<!-- Hardware design source license: Creative Commons BY-SA-NC (see microduck_rl README). -->',
        '<!-- Source MJCF: https://github.com/pollen-robotics/microduck_rl/blob/main/src/mjlab_microduck/robot/microduck/robot_allcollisions.xml -->',
        '<!-- Full collision geometry; MuJoCo collision masks and actuator dynamics are not reproduced. -->',
        '<!-- Actuators, sensors, sites, cameras, and root world placement are omitted. -->',
        '<!-- Joint effort uses chosen_actuator +/-0.96; velocity 1.0 rad/s is a non-source URDF fallback, not physics-ready. -->',
        '<robot name="microduck">',
        *links,
        *joints,
        '</robot>',
        '',
    ]
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    MESH_OUTPUT.mkdir(parents=True, exist_ok=True)
    for stale in MESH_OUTPUT.glob("*.stl"):
        if stale.name not in copied:
            stale.unlink()
    for filename in sorted(copied):
        shutil.copyfile(ASSET_DIR / filename, MESH_OUTPUT / filename)
    OUTPUT.write_text("\n".join(output_xml), encoding="utf-8", newline="\n")
    link_count = sum(line.startswith('  <link ') for line in links)
    print(f"Generated {OUTPUT.relative_to(PROJECT)}: {link_count} links, {len(joints)} joints, {visual_count} visuals, {len(copied)} unique meshes.")


if __name__ == "__main__":
    main()
