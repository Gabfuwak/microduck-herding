#!/usr/bin/env python3
"""Validate native loaders and optionally an actual Unity Android APK.

Runs the Linux loader, cross-compiles its Android ARM64 counterpart, and audits
all native libraries. Android execution requires real ARM64 Android hardware;
successful cross-compilation alone is not reported as runtime validation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile
import zipfile

import prepare

ROOT = prepare.ROOT
HERE = Path(__file__).resolve().parent
LIBRARIES = ("libonnxruntime.so", "libmujoco.so", "libmicroduck_control.so")


def run(args: list[str]) -> str:
    return subprocess.check_output(args, text=True, stderr=subprocess.STDOUT).strip()


def android_tools(explicit: Path | None) -> Path:
    if explicit:
        return explicit
    text = (ROOT / "ProjectSettings/ProjectVersion.txt").read_text()
    version = re.search(r"^m_EditorVersion: (\S+)", text, re.M).group(1)
    return Path.home() / "Unity/Hub/Editor" / version / "Editor/Data/PlaybackEngines/AndroidPlayer"


def inspect(path: Path, platform: str) -> dict:
    # MuJoCo has a different ABI from the two Pollen integration libraries.
    if path.name != "libmujoco.so":
        return prepare.inspect_elf(path, platform, core=path.name == "libmicroduck_control.so")
    header = run(["readelf", "-h", str(path)])
    machine = "AArch64" if platform == "android-arm64" else "Advanced Micro Devices X86-64"
    if machine not in header:
        raise RuntimeError(f"Wrong MuJoCo architecture: {path}")
    dynamic = run(["readelf", "-d", str(path)])
    needed = re.findall(r"\(NEEDED\).*\[(.*?)\]", dynamic)
    aligns = [int(line.split()[-1], 16) for line in run(["readelf", "-lW", str(path)]).splitlines() if line.strip().startswith("LOAD ")]
    if platform == "android-arm64" and (min(aligns) < 16384 or set(needed) - {"libc.so", "libdl.so", "libm.so", "liblog.so"}):
        raise RuntimeError(f"MuJoCo Android alignment/dependency check failed: {path}")
    return {"machine": machine, "needed": needed, "load_alignments": aligns}


def check_plugin_meta(platform: str, path: Path) -> None:
    text = Path(str(path) + ".meta").read_text()
    segments = text.split("  - first:")
    enabled = [segment for segment in segments if "enabled: 1" in segment]
    if platform == "android-arm64":
        if len(enabled) != 1 or "Android: Android" not in enabled[0] or "CPU: ARM64" not in enabled[0]:
            raise RuntimeError(f"Expected Android-only ARM64 plugin: {path}")
    else:
        if len(enabled) != 2 or not any("Editor: Editor" in s and "OS: Linux" in s for s in enabled) or not any("Standalone: Linux64" in s for s in enabled):
            raise RuntimeError(f"Expected Linux x86_64 Editor and Player plugin: {path}")


def loadable_image(payload: bytes) -> list[dict]:
    """Fingerprint ELF64 LOAD segments while allowing non-runtime symbol stripping."""
    if payload[:6] != b"\x7fELF\x02\x01":
        raise RuntimeError("Expected little-endian ELF64 native library.")
    header = bytearray(payload)
    # strip may move/remove section headers and non-allocated debug/symbol tables;
    # these header fields do not change the image loaded by Android's linker.
    header[40:48] = bytes(8)
    header[58:64] = bytes(6)
    phoff = struct.unpack_from("<Q", payload, 32)[0]
    phentsize, phnum = struct.unpack_from("<HH", payload, 54)
    segments = []
    for index in range(phnum):
        kind, flags, offset, address, _, size, memory, alignment = struct.unpack_from("<IIQQQQQQ", payload, phoff + index * phentsize)
        if kind == 1:
            segments.append({"flags": flags, "offset": offset, "address": address, "size": size,
                             "memory": memory, "alignment": alignment,
                             "sha256": hashlib.sha256(header[offset:offset + size]).hexdigest()})
    return segments


def check_apk(apk: Path, tools: Path) -> dict:
    report = {"path": str(apk), "sha256": hashlib.sha256(apk.read_bytes()).hexdigest(), "native_libraries": []}
    with zipfile.ZipFile(apk) as archive, apk.open("rb") as stream, tempfile.TemporaryDirectory(prefix="pollen-apk-") as temporary:
        if "lib/arm64-v8a/libil2cpp.so" not in archive.namelist():
            raise RuntimeError("APK does not contain an ARM64 IL2CPP player.")
        for name in LIBRARIES:
            info = archive.getinfo("lib/arm64-v8a/" + name)
            payload = archive.read(info)
            expected = (ROOT / "Assets/Plugins/Android/arm64-v8a" / name).read_bytes()
            if loadable_image(payload) != loadable_image(expected):
                raise RuntimeError(f"APK loadable native image differs from the verified packaged artifact: {name}")
            path = Path(temporary) / name
            path.write_bytes(payload)
            record = {"name": name, "sha256": hashlib.sha256(payload).hexdigest(), "elf": inspect(path, "android-arm64"), "loadable_image_matches": True, "byte_identical": payload == expected}
            # Android can mmap uncompressed libraries only when their ZIP data offsets
            # meet the device's page alignment. Compressed entries are extracted.
            stream.seek(info.header_offset)
            header = stream.read(30)
            filename_length, extra_length = struct.unpack_from("<HH", header, 26)
            offset = info.header_offset + 30 + filename_length + extra_length
            record.update(zip_compression=info.compress_type, zip_data_offset=offset)
            if info.compress_type == zipfile.ZIP_STORED and offset % 16384:
                raise RuntimeError(f"APK stores {name} at an offset not aligned to 16 KiB: {offset}")
            report["native_libraries"].append(record)
    zipaligns = sorted((tools / "SDK/build-tools").glob("*/zipalign"))
    if not zipaligns:
        raise RuntimeError("Android SDK zipalign missing.")
    report["zipalign"] = run([str(zipaligns[-1]), "-c", "-P", "16", "4", str(apk)]) or "passed"
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--android-tools", type=Path, help="Unity AndroidPlayer directory containing SDK and NDK")
    parser.add_argument("--apk", type=Path, help="Also inspect a built Unity Android APK")
    options = parser.parse_args()
    run([sys.executable, str(HERE / "prepare.py"), "--verify"])
    output = ROOT / "Builds/PollenRuntime"
    output.mkdir(parents=True, exist_ok=True)
    report = {"schema_version": 1, "native_libraries": [], "android_runtime_executed": False}
    for platform, directory in prepare.PLATFORMS.items():
        for name in LIBRARIES:
            path = ROOT / "Assets/Plugins" / directory / name
            check_plugin_meta(platform, path)
            report["native_libraries"].append({"platform": platform, "path": str(path.relative_to(ROOT)), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "elf": inspect(path, platform)})
    linux = output / "runtime_loader-linux"
    run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2", str(HERE / "runtime_loader.c"), "-ldl", "-o", str(linux)])
    report["linux_loader"] = json.loads(run([str(linux), *[str(ROOT / "Assets/Plugins/Linux/x86_64" / name) for name in LIBRARIES]]))
    tools = android_tools(options.android_tools)
    compiler = tools / "NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/aarch64-linux-android34-clang"
    android = output / "runtime_loader-android-arm64"
    run([str(compiler), "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2", "-fPIE", "-pie", "-Wl,-z,max-page-size=16384", str(HERE / "runtime_loader.c"), "-ldl", "-o", str(android)])
    report["android_loader_crosscompiled"] = str(android.relative_to(ROOT))
    if options.apk:
        report["apk"] = check_apk(options.apk, tools)
    (output / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"linux_loader": report["linux_loader"], "native_libraries_checked": len(report["native_libraries"]), "android_loader_crosscompiled": True, "android_runtime_executed": False, "apk_checked": bool(options.apk), "report": str(output / "validation.json")}))


if __name__ == "__main__":
    main()
