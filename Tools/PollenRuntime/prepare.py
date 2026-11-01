#!/usr/bin/env python3
"""Reproduce or verify the checked-in Pollen policies and native inference runtime.

Normal Unity use needs no download. --verify checks the checked-in payloads.
Without --verify, fetch pinned archives into Library/PollenRuntime and reproduce
the assets. Native control libraries are built separately by Tools/PollenControl.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path
import re
import subprocess
import tarfile
import urllib.request
import uuid
import zipfile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
NAMESPACE = uuid.UUID("1edd0f3c-1439-46ac-9bfa-d9d4f9a283c1")
PLATFORMS = {"linux-x86_64": "Linux/x86_64", "android-arm64": "Android/arm64-v8a"}


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def checked(data: bytes, expected: str, label: str) -> bytes:
    actual = digest(data)
    if actual != expected:
        raise RuntimeError(f"SHA-256 mismatch for {label}: expected {expected}, got {actual}")
    return data


def meta(path: Path, platform: str | None = None) -> None:
    target = Path(str(path) + ".meta")
    if target.exists():
        return
    guid = uuid.uuid5(NAMESPACE, path.relative_to(ROOT).as_posix()).hex
    prefix = f"fileFormatVersion: 2\nguid: {guid}\n"
    if platform:
        template = (ROOT / "Assets/Plugins" / PLATFORMS[platform] / "libmujoco.so.meta").read_text()
        content = re.sub(r"(?m)^guid: [0-9a-f]+$", f"guid: {guid}", template)
        content = "\n".join(line.rstrip() for line in content.splitlines()) + "\n"
    elif path.is_dir():
        content = prefix + "folderAsset: yes\nDefaultImporter:\n  externalObjects: {}\n  userData:\n  assetBundleName:\n  assetBundleVariant:\n"
    else:
        importer = "TextScriptImporter" if path.suffix in {".bytes", ".txt", ".json", ".xml"} else "DefaultImporter"
        content = prefix + importer + ":\n  externalObjects: {}\n  userData:\n  assetBundleName:\n  assetBundleVariant:\n"
    target.write_text(content)


def install(destination: str, data: bytes, platform: str | None = None) -> None:
    path = ROOT / destination
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.read_bytes() == data:
        pass
    else:
        temporary = path.with_name(path.name + ".partial")
        temporary.write_bytes(data)
        temporary.replace(path)
    if platform:
        path.chmod(0o755)
    if destination.startswith("Assets/"):
        for parent in reversed(path.parents):
            if parent != ROOT / "Assets" and parent.is_relative_to(ROOT / "Assets"):
                meta(parent)
        meta(path, platform)


def archive_member(data: bytes, format: str, member: str | None) -> bytes:
    if format == "raw":
        return data
    if format == "zip":
        with zipfile.ZipFile(io.BytesIO(data)) as archive:
            return archive.read(member)
    if format == "tgz":
        with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as archive:
            stream = archive.extractfile(member)
            if stream is None:
                raise RuntimeError(f"Missing archive file: {member}")
            return stream.read()
    raise ValueError(f"Unknown archive format: {format}")


def inspect_elf(path: Path, platform: str, *, core: bool = False) -> dict:
    def readelf(*args: str) -> str:
        return subprocess.check_output(["readelf", *args, str(path)], text=True)
    header = readelf("-h")
    machine = "AArch64" if platform == "android-arm64" else "Advanced Micro Devices X86-64"
    if machine not in header or not re.search(r"Type:\s+DYN", header):
        raise RuntimeError(f"Expected {platform} shared object: {path}")
    dynamic = readelf("-d")
    needed = re.findall(r"\(NEEDED\).*\[(.*?)\]", dynamic)
    soname = re.search(r"\(SONAME\).*\[(.*?)\]", dynamic)
    alignments = [int(line.split()[-1], 16) for line in readelf("-lW").splitlines() if line.strip().startswith("LOAD ")]
    if platform == "android-arm64":
        if not alignments or min(alignments) < 16384:
            raise RuntimeError(f"Android ELF lacks 16 KiB LOAD alignment: {path}")
        allowed = {"libc.so", "libdl.so", "libm.so", "liblog.so", "libandroid.so", "libonnxruntime.so"}
        if set(needed) - allowed:
            raise RuntimeError(f"Unexpected Android shared dependencies in {path}: {needed}")
    symbols = readelf("--dyn-syms", "--wide")
    required = ["md_create", "md_destroy", "md_step", "md_reset", "md_last_error", "md_abi_version"] if core else ["OrtGetApiBase"]
    for symbol in required:
        if not re.search(rf"\b{symbol}(?:@@?\S+)?$", symbols, re.M):
            raise RuntimeError(f"Missing native ABI export {symbol}: {path}")
    return {"machine": machine, "soname": soname.group(1) if soname else None,
            "needed": needed, "load_alignments": alignments, "required_exports": required}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify", action="store_true", help="Verify checked-in artifacts without downloading")
    parser.add_argument("--cache", type=Path, default=ROOT / "Library/PollenRuntime")
    parser.add_argument("--core-linux", type=Path)
    parser.add_argument("--core-android", type=Path)
    parser.add_argument("--core-provenance", type=Path, default=ROOT / "Builds/PollenControl/provenance.json",
                        help="Build provenance matching newly supplied core libraries")
    options = parser.parse_args()
    lock = json.loads((HERE / "runtime.lock.json").read_text())
    report = {"schema_version": 1, "onnxruntime_version": lock["onnxruntime_version"],
              "policy_revision": lock["policy_revision"], "artifacts": []}
    for source in lock["sources"]:
        downloaded = None
        if not options.verify:
            cached = options.cache / source["cache_name"]
            cached.parent.mkdir(parents=True, exist_ok=True)
            if not cached.exists():
                request = urllib.request.Request(source["url"], headers={"User-Agent": "MicroDuck-PollenRuntime/1"})
                with urllib.request.urlopen(request, timeout=180) as response:
                    downloaded = response.read()
                checked(downloaded, source["sha256"], source["url"])
                cached.write_bytes(downloaded)
            downloaded = checked(cached.read_bytes(), source["sha256"], str(cached))
        for artifact in source["artifacts"]:
            destination = ROOT / artifact["destination"]
            if not options.verify:
                data = archive_member(downloaded, source["format"], artifact.get("member"))
                checked(data, artifact["sha256"], artifact["destination"])
                install(artifact["destination"], data, artifact.get("platform"))
            checked(destination.read_bytes(), artifact["sha256"], str(destination))
            record = {**artifact, "source_url": source["url"], "source_sha256": source["sha256"]}
            if artifact.get("platform"):
                record["elf"] = inspect_elf(destination, artifact["platform"])
            report["artifacts"].append(record)
    core_lock_path = HERE / "control-artifacts.json"
    core_lock = json.loads(core_lock_path.read_text()) if core_lock_path.exists() else {"schema_version": 1, "artifacts": []}
    pending_core = []
    for platform, supplied in (("linux-x86_64", options.core_linux), ("android-arm64", options.core_android)):
        if supplied:
            elf = inspect_elf(supplied, platform, core=True)
            data = supplied.read_bytes()
            relative = f"Assets/Plugins/{PLATFORMS[platform]}/libmicroduck_control.so"
            pending_core.append((relative, data, platform))
            core_lock["artifacts"] = [a for a in core_lock["artifacts"] if a["platform"] != platform]
            core_lock["artifacts"].append({"destination": relative, "platform": platform, "sha256": digest(data), "elf": elf})
    provenance_path = HERE / "control-provenance.json"
    if options.core_linux or options.core_android:
        provenance = json.loads(options.core_provenance.read_text())
        for artifact in core_lock["artifacts"]:
            if provenance["artifacts"][artifact["platform"]]["sha256"] != artifact["sha256"]:
                raise RuntimeError("Core build provenance does not match the supplied artifacts")
        for destination, data, platform in pending_core:
            install(destination, data, platform)
        core_lock_path.write_text(json.dumps(core_lock, indent=2) + "\n")
        provenance_path.write_text(json.dumps(provenance, indent=2) + "\n")
    if options.verify and (len(core_lock["artifacts"]) != 2 or
                           {a["platform"] for a in core_lock["artifacts"]} != set(PLATFORMS)):
        raise RuntimeError("Verification requires both Linux x86_64 and Android ARM64 control libraries")
    for artifact in core_lock["artifacts"]:
        native = ROOT / artifact["destination"]
        checked(native.read_bytes(), artifact["sha256"], str(native))
        inspect_elf(native, artifact["platform"], core=True)
    if core_lock["artifacts"]:
        provenance = json.loads(provenance_path.read_text())
        for artifact in core_lock["artifacts"]:
            if provenance["artifacts"][artifact["platform"]]["sha256"] != artifact["sha256"]:
                raise RuntimeError("Checked-in core provenance is inconsistent with the packaged library")
    if not options.verify:
        (HERE / "provenance.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"Verified {len(report['artifacts'])} pinned runtime/model/notice assets and {len(core_lock['artifacts'])} native control libraries.")


if __name__ == "__main__":
    main()
