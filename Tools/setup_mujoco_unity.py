#!/usr/bin/env python3
"""Vendor pinned MuJoCo Unity bindings, Linux native library and MicroDuck VFS model.

Run from any directory with Python 3.10+: python3 Tools/setup_mujoco_unity.py
The script uses the same source checkout and commit guard as build_mujoco_android.py.
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


def make_model() -> None:
    source_xml = ROBOT_DIR / "robot_allcollisions.xml"
    scene_xml = ROBOT_DIR / "scene.xml"
    if not source_xml.is_file() or not scene_xml.is_file():
        raise RuntimeError(f"Missing pinned MicroDuck source MJCF under {ROBOT_DIR}")
    robot = ET.parse(source_xml).getroot()
    scene = ET.parse(scene_xml).getroot()
    # Expand the source scene's include in-place, preserving all robot XML
    # elements. Keep scene lighting/visual settings but discard texture assets.
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
    # Remove any source keyframes while retaining all source defaults, bodies,
    # contact, actuators, sensors, equality and native mesh/material assets.
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
    # Only source meshes referenced by a geom are bundled. Compiler meshdir
    # remains "assets" and VFS filenames therefore match assets/*.stl.
    referenced_mesh_names = {geom.get("mesh") for geom in robot.iter("geom") if geom.get("mesh")}
    mesh_files = sorted({
        mesh.get("file")
        for mesh in robot.findall("./asset/mesh")
        if mesh.get("file") and (mesh.get("name") or Path(mesh.get("file", "")).stem) in referenced_mesh_names
    })
    output_asset_dir = MODEL_OUT / "assets"
    output_asset_dir.mkdir(parents=True, exist_ok=True)
    for filename in mesh_files:
        relative = PurePosixPath(filename)
        if relative.is_absolute() or ".." in relative.parts or len(relative.parts) != 1:
            raise RuntimeError(f"Unexpected source mesh path: {filename}")
        source_mesh = ROBOT_DIR / "assets" / relative.name
        if not source_mesh.is_file():
            raise RuntimeError(f"Referenced mesh missing: {source_mesh}")
        shutil.copyfile(source_mesh, output_asset_dir / f"{relative.name}.bytes")
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
