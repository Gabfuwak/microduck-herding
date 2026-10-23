#!/usr/bin/env python3
"""Build the pinned MuJoCo 3.15.0 Android ARM64 shared library.

Optional for a project opened with the checked-in native libraries. Run from
the Unity project root (or any working directory):
    python3 Tools/build_mujoco_android.py [--ndk PATH]
Requires Git, CMake and an Android NDK (Unity's Android Build Support supplies
the SDK/NDK/OpenJDK needed for player builds). Resolve the NDK from the Unity
6000.6.3f1 installation, ANDROID_NDK_ROOT, or --ndk. Source and CMake caches
live under ignored Library/MuJoCoAndroid; the output replaces the Android
ARM64 plugin and its native license notices.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
CACHE = PROJECT / "Library/MuJoCoAndroid"
SOURCE = CACHE / "source"
BUILD = CACHE / "build"
PIN = "9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5"
TAG = "3.15.0"
API = 34
ABI = "arm64-v8a"
OUTPUT = PROJECT / "Assets/Plugins/Android/arm64-v8a/libmujoco.so"
LICENSE_OUTPUT = PROJECT / "Assets/StreamingAssets/NativeLicenses/MuJoCo"


def run(args: list[str], *, cwd: Path | None = None) -> str:
    """Run a command and return stdout, raising a useful error on failure."""
    try:
        result = subprocess.run(args, cwd=cwd, check=True, text=True, capture_output=True)
    except FileNotFoundError as exc:
        raise RuntimeError(f"Required executable not found: {args[0]}") from exc
    except subprocess.CalledProcessError as exc:
        detail = (exc.stderr or exc.stdout).strip()
        raise RuntimeError(f"Command failed ({' '.join(args)}): {detail}") from exc
    return result.stdout.strip()


def resolve_ndk(explicit: str | None) -> Path:
    """Resolve an explicit, environment, or Unity-version-matched NDK."""
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    elif os.environ.get("ANDROID_NDK_ROOT"):
        candidates.append(Path(os.environ["ANDROID_NDK_ROOT"]).expanduser())
    else:
        version_file = PROJECT / "ProjectSettings/ProjectVersion.txt"
        match = re.search(r"^m_EditorVersion:\s*(\S+)", version_file.read_text(), re.M)
        if not match:
            raise RuntimeError(f"Cannot read Unity editor version from {version_file}")
        version = match.group(1)
        hub = Path(os.environ.get("UNITY_HUB", Path.home() / "Unity/Hub"))
        candidates.extend((hub / "Editor" / version / "Editor/Data/PlaybackEngines/AndroidPlayer/NDK",
                           Path("/opt/unity/Editor") / version / "Editor/Data/PlaybackEngines/AndroidPlayer/NDK"))
    for candidate in candidates:
        if (candidate / "source.properties").is_file() and (candidate / "toolchains/llvm/prebuilt").is_dir():
            return candidate.resolve()
    tried = ", ".join(map(str, candidates))
    raise RuntimeError(f"Android NDK not found (checked {tried}); pass --ndk or set ANDROID_NDK_ROOT")


def ensure_source() -> str:
    """Obtain only a missing checkout and reject any unpinned existing checkout."""
    if SOURCE.exists():
        if not (SOURCE / ".git").exists():
            raise RuntimeError(f"Refusing to replace existing non-git source directory: {SOURCE}")
    else:
        CACHE.mkdir(parents=True, exist_ok=True)
        run(["git", "clone", "--branch", f"{TAG}", "--depth", "1",
             "https://github.com/google-deepmind/mujoco.git", str(SOURCE)])
    commit = run(["git", "-C", str(SOURCE), "rev-parse", "HEAD"])
    if commit != PIN:
        raise RuntimeError(f"MuJoCo source pin mismatch: expected {PIN}, found {commit}; source left untouched")
    return commit


def tool_path(ndk: Path, name: str) -> Path:
    """Find an NDK LLVM tool in the single host prebuilt directory."""
    prebuilt = ndk / "toolchains/llvm/prebuilt"
    hosts = sorted(p for p in prebuilt.iterdir() if p.is_dir())
    for host in hosts:
        path = host / "bin" / name
        if path.is_file():
            return path
    raise RuntimeError(f"NDK tool {name} not found under {prebuilt}")


def licenses(ndk: Path) -> list[dict[str, str]]:
    """Bundle the notices for compiled dependencies and the static C++ runtime."""
    dependencies = {
        "ccd": ("BSD-LICENSE",),
        "lodepng": ("LICENSE",),
        "marchingcubecpp": ("README.md",),  # Author's public-domain/MIT grant is here.
        "miniz": ("LICENSE",),
        "qhull": ("COPYING.txt",),
        "tinyobjloader": ("LICENSE", "mapbox/LICENSE"),
        "tinyxml2": ("LICENSE.txt",),
    }
    notices = [("MuJoCo", SOURCE / "LICENSE", "MuJoCo-LICENSE")]
    for name, filenames in dependencies.items():
        for filename in filenames:
            source = BUILD / "_deps" / f"{name}-src" / filename
            notices.append((name, source, f"{name}-{filename.replace('/', '-')}"))
    notices.append(("Android NDK static runtime", ndk / "NOTICE.toolchain", "NDK-NOTICE.toolchain"))
    for _, source, _ in notices:
        if not source.is_file():
            raise RuntimeError(f"Required redistribution notice is missing: {source}")
    LICENSE_OUTPUT.mkdir(parents=True, exist_ok=True)
    copied = []
    for name, source, filename in notices:
        shutil.copy2(source, LICENSE_OUTPUT / filename)
        copied.append({"name": name, "file": filename})
    glad = SOURCE / "src/render/classic/glad/glad.c"
    (LICENSE_OUTPUT / "GLAD-NOTICE.txt").write_text(
        "\n".join(glad.read_text().splitlines()[:18]) + "\n")
    copied.append({"name": "GLAD", "file": "GLAD-NOTICE.txt"})
    return copied


def inspect_artifact(artifact: Path, ndk: Path) -> tuple[dict[str, object], list[str]]:
    """Validate Android ELF identity, 16 KiB segments, dependencies and exports."""
    readelf = tool_path(ndk, "llvm-readelf")
    nm = tool_path(ndk, "llvm-nm")
    header = run([str(readelf), "-h", str(artifact)])
    if "AArch64" not in header or "DYN (Shared object file)" not in header:
        raise RuntimeError("Built artifact is not an AArch64 ELF shared object")
    dynamic = run([str(readelf), "-d", str(artifact)])
    soname = re.search(r"\(SONAME\).*\[(.*?)\]", dynamic)
    if not soname or soname.group(1) != "libmujoco.so":
        raise RuntimeError(f"Unexpected or missing SONAME: {soname.group(1) if soname else 'none'}")
    needed = re.findall(r"\(NEEDED\).*\[(.*?)\]", dynamic)
    allowed = {"libc.so", "libm.so", "libdl.so", "liblog.so", "libandroid.so", "libz.so"}
    leaked = sorted(set(needed) - allowed)
    if leaked:
        raise RuntimeError(f"Unexpected non-platform/host shared dependencies: {', '.join(leaked)}")
    program = run([str(readelf), "-lW", str(artifact)])
    aligns = [int(line.split()[-1], 16) for line in program.splitlines()
              if line.split() and line.split()[0] == "LOAD"]
    if not aligns or min(aligns) < 0x4000:
        raise RuntimeError("ELF LOAD segments are not all aligned to at least 16 KiB")
    symbols = run([str(nm), "-D", "--defined-only", str(artifact)])
    exported = sorted(set(re.findall(r"\b(mj_[A-Za-z0-9_]+)$", symbols, re.M)))
    required = {"mj_versionString", "mj_loadXML", "mj_step"}
    missing = sorted(required - set(exported))
    if missing:
        raise RuntimeError(f"Expected MuJoCo API symbols missing: {', '.join(missing)}")
    return ({"elf_machine": "AArch64", "soname": soname.group(1), "needed": needed,
             "load_segment_alignments": [f"0x{x:x}" for x in aligns],
             "exported_mujoco_symbol_count": len(exported)}, exported)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ndk", help="Android NDK root (otherwise resolve from Unity version/environment)")
    args = parser.parse_args()
    ndk = resolve_ndk(args.ndk)
    commit = ensure_source()
    cmake = shutil.which("cmake")
    ninja = shutil.which("ninja")
    if not cmake or not ninja:
        raise RuntimeError("Both cmake and ninja must be available on PATH")
    flags = "-Wl,-z,max-page-size=16384 -Wl,-z,common-page-size=16384"
    configure = [cmake, "-S", str(SOURCE), "-B", str(BUILD), "-G", "Ninja",
                 "-DCMAKE_PLATFORM_NO_VERSIONED_SONAME=ON",
                 f"-DCMAKE_TOOLCHAIN_FILE={ndk / 'build/cmake/android.toolchain.cmake'}",
                 f"-DANDROID_ABI={ABI}", f"-DANDROID_PLATFORM=android-{API}",
                 "-DANDROID_STL=c++_static",
                 "-DCMAKE_BUILD_TYPE=Release", "-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF",
                 "-DCMAKE_POLICY_VERSION_MINIMUM=3.16", f"-DCMAKE_SHARED_LINKER_FLAGS={flags}",
                 "-DMUJOCO_BUILD_EXAMPLES=OFF", "-DMUJOCO_BUILD_SIMULATE=OFF",
                 "-DMUJOCO_BUILD_STUDIO=OFF", "-DMUJOCO_BUILD_TESTS=OFF",
                 "-DMUJOCO_TEST_PYTHON_UTIL=OFF", "-DMUJOCO_WITH_USD=OFF",
                 "-DMUJOCO_USE_FILAMENT=OFF", "-DMUJOCO_USE_FILAMENT_MJR_COMPAT=OFF",
                 "-DMUJOCO_ENABLE_AVX=OFF", "-DMUJOCO_ENABLE_AVX_INTRINSICS=OFF",
                 "-DMUJOCO_ENABLE_LTO=OFF"]
    run(configure)
    run([cmake, "--build", str(BUILD), "--target", "mujoco", "--parallel", str(os.cpu_count() or 1)])
    candidates = [BUILD / "libmujoco.so"]
    built = next((p for p in candidates if p.is_file()), None)
    if built is None:
        matches = list(BUILD.rglob("libmujoco.so"))
        built = matches[0] if len(matches) == 1 else None
    if built is None:
        raise RuntimeError(f"Could not uniquely locate built libmujoco.so under {BUILD}")
    strip = tool_path(ndk, "llvm-strip")
    run([str(strip), "--strip-debug", str(built)])
    inspection, symbols = inspect_artifact(built, ndk)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(built, OUTPUT)
    notices = licenses(ndk)
    version = (ndk / "source.properties").read_text()
    ndk_version = re.search(r"^Pkg.Revision\s*=\s*(.+)$", version, re.M)
    report = {"source": {"repository": "https://github.com/google-deepmind/mujoco.git", "tag": TAG,
                          "commit": commit},
              "toolchain": {"ndk": str(ndk), "ndk_revision": ndk_version.group(1).strip() if ndk_version else "unknown",
                            "cmake": run([cmake, "--version"]).splitlines()[0],
                            "ninja": run([ninja, "--version"])},
              "configuration": {"abi": ABI, "android_api": API, "build_type": "Release",
                                "stl": "c++_static", "lto": False, "linker_flags": flags,
                                "cmake_options": configure[5:]},
              "artifact": {"path": str(OUTPUT.relative_to(PROJECT)),
                           "sha256": hashlib.sha256(OUTPUT.read_bytes()).hexdigest(), **inspection,
                           "exported_symbols": symbols},
              "licenses": notices}
    (CACHE / "build-report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"Built {OUTPUT.relative_to(PROJECT)} (SHA-256 {report['artifact']['sha256']})")
    print(f"Build report: {(CACHE / 'build-report.json').relative_to(PROJECT)}")
    print(f"License bundle: {LICENSE_OUTPUT.relative_to(PROJECT)}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError) as error:
        print(f"build_mujoco_android: error: {error}", file=sys.stderr)
        sys.exit(1)
