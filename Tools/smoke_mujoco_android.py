#!/usr/bin/env python3
"""Build and run the pinned MuJoCo native smoke test on an Android ARM64 device.

Usage: python3 Tools/smoke_mujoco_android.py [--ndk NDK] [--serial SERIAL]
       python3 Tools/smoke_mujoco_android.py --build-only [--ndk NDK]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shlex
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from build_mujoco_android import resolve_ndk

PROJECT = Path(__file__).resolve().parents[1]
SOURCE = PROJECT / "Library/MuJoCoAndroid/source"
INCLUDE = SOURCE / "include"
PROGRAM = PROJECT / "Tools/mujoco_android_smoke.c"
LIBRARY = PROJECT / "Assets/Plugins/Android/arm64-v8a/libmujoco.so"
OUTPUT = PROJECT / "Library/MuJoCoAndroid/smoke/mujoco-android-smoke"
RESULT = PROJECT / "Library/MuJoCoAndroid/android-smoke-result.json"
REMOTE_ROOT = "/data/local/tmp/mujoco-native-smoke"


class SmokeError(RuntimeError):
    """An actionable build, adb, or smoke-test failure."""


def compile_smoke(ndk: Path) -> None:
    """Compile a position-independent ARM64 Android API 34 executable."""
    compiler = ndk / "toolchains/llvm/prebuilt/linux-x86_64/bin/aarch64-linux-android34-clang"
    if not PROGRAM.is_file() or not (INCLUDE / "mujoco/mujoco.h").is_file():
        raise SmokeError(f"Smoke source or pinned MuJoCo header is missing: {PROGRAM} / {INCLUDE}")
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    command = [
        str(compiler), "-std=c11", "-O2", "-fPIE", "-pie", "-Wall", "-Wextra",
        "-Wl,-z,max-page-size=16384", "-Wl,-z,common-page-size=16384",
        f"-I{INCLUDE}", str(PROGRAM), "-ldl", "-lm", "-o", str(OUTPUT),
    ]
    completed = subprocess.run(command, text=True, capture_output=True, check=False)
    if completed.returncode:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise SmokeError(f"Android ARM64 smoke executable compilation failed:\n{detail}")
    OUTPUT.chmod(0o755)
    print(f"Built {OUTPUT.relative_to(PROJECT)}")


def find_adb(ndk: Path) -> Path:
    """Find adb alongside the Unity-installed NDK or on PATH."""
    sdk_adb = ndk.parent / "SDK/platform-tools/adb"
    if sdk_adb.is_file():
        return sdk_adb
    from_path = shutil.which("adb")
    if from_path:
        return Path(from_path)
    raise SmokeError("adb was not found; install Android platform-tools or pass an NDK with sibling SDK")


def select_device(adb: Path, serial: str | None) -> str:
    """Require a single authorized connected device unless --serial selects one."""
    completed = subprocess.run([str(adb), "devices"], text=True, capture_output=True, check=False)
    if completed.returncode:
        raise SmokeError(f"adb devices failed: {completed.stderr.strip()}")
    devices: dict[str, str] = {}
    for line in completed.stdout.splitlines()[1:]:
        fields = line.split()
        if len(fields) >= 2:
            devices[fields[0]] = fields[1]
    if serial:
        state = devices.get(serial)
        if state != "device":
            if state == "unauthorized":
                raise SmokeError(f"Android device {serial} is connected but unauthorized; unlock Quest and accept its USB debugging prompt")
            if state == "offline":
                raise SmokeError(f"Android device {serial} is offline; reconnect USB and authorize it")
            raise SmokeError(f"Android device {serial} is not connected/authorized (adb state: {state or 'not listed'})")
        return serial
    authorized = [device for device, state in devices.items() if state == "device"]
    if len(authorized) == 1:
        return authorized[0]
    if len(authorized) > 1:
        raise SmokeError(f"Multiple authorized Android devices ({', '.join(authorized)}); select one with --serial")
    if "unauthorized" in devices.values():
        raise SmokeError("Android device is connected but unauthorized; unlock Quest and accept the USB debugging prompt")
    if "offline" in devices.values():
        raise SmokeError("Android device is offline; reconnect USB and authorize it")
    raise SmokeError("No Android device connected; connect and authorize the Quest over USB")


def adb_run(adb: Path, serial: str, *arguments: str) -> subprocess.CompletedProcess[str]:
    """Run adb for the selected device and convert failures into useful errors."""
    completed = subprocess.run(
        [str(adb), "-s", serial, *arguments], text=True, capture_output=True, check=False
    )
    if completed.returncode:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise SmokeError(f"adb {' '.join(arguments[:2])} failed: {detail}")
    return completed


def device_property(adb: Path, serial: str, name: str) -> str:
    """Read one Android system property."""
    return adb_run(adb, serial, "shell", "getprop", name).stdout.strip()


def run_on_device(adb: Path, serial: str) -> dict[str, object]:
    """Stage only this run's files, execute the real library smoke, and clean up."""
    if not LIBRARY.is_file():
        raise SmokeError(f"Built plug-in is missing: {LIBRARY}; run the MuJoCo Android native build first")
    if not OUTPUT.is_file():
        raise SmokeError(f"Smoke executable is missing: {OUTPUT}; run this script with --build-only first")
    run_name = f"run-{os.getpid()}-{os.urandom(6).hex()}"
    remote_dir = f"{REMOTE_ROOT}/{run_name}"
    created = False
    try:
        adb_run(adb, serial, "shell", "mkdir", "-p", remote_dir)
        created = True
        adb_run(adb, serial, "push", str(LIBRARY), f"{remote_dir}/libmujoco.so")
        adb_run(adb, serial, "push", str(OUTPUT), f"{remote_dir}/mujoco-android-smoke")
        adb_run(adb, serial, "shell", "chmod", "700", f"{remote_dir}/mujoco-android-smoke")
        shell_command = (
            f"LD_LIBRARY_PATH={shlex.quote(remote_dir)} "
            f"{shlex.quote(remote_dir + '/mujoco-android-smoke')} "
            f"{shlex.quote(remote_dir + '/libmujoco.so')}"
        )
        execution = adb_run(adb, serial, "shell", shell_command)
        try:
            smoke = json.loads(execution.stdout.strip())
        except json.JSONDecodeError as exc:
            raise SmokeError(f"Native smoke returned no valid JSON; stdout={execution.stdout!r}, stderr={execution.stderr!r}") from exc
        if not isinstance(smoke, dict) or smoke.get("version") != "3.15.0":
            raise SmokeError(f"Native smoke returned unexpected result: {smoke!r}")
        props = {
            "model": device_property(adb, serial, "ro.product.model"),
            "android_sdk": device_property(adb, serial, "ro.build.version.sdk"),
            "abi": device_property(adb, serial, "ro.product.cpu.abi"),
        }
        document: dict[str, object] = {
            "device": {"serial": serial, **props},
            "library": {
                "path": str(LIBRARY.relative_to(PROJECT)),
                "sha256": hashlib.sha256(LIBRARY.read_bytes()).hexdigest(),
            },
            "smoke": smoke,
            "completed_at_utc": datetime.now(timezone.utc).isoformat(),
        }
        RESULT.parent.mkdir(parents=True, exist_ok=True)
        RESULT.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(document, indent=2, sort_keys=True))
        print(f"Saved {RESULT.relative_to(PROJECT)}")
        return document
    finally:
        if created:
            cleanup = subprocess.run(
                [str(adb), "-s", serial, "shell", "rm", "-rf", remote_dir],
                text=True, capture_output=True, check=False,
            )
            if cleanup.returncode:
                print(f"Warning: could not remove this run's staged files {remote_dir}: {cleanup.stderr.strip()}", file=sys.stderr)


def main() -> int:
    """Build the test and optionally run it on an authorized Android device."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ndk", help="Android NDK path (defaults to ANDROID_NDK_ROOT or Unity's installed NDK)")
    parser.add_argument("--serial", help="adb device serial when more than one device is connected")
    parser.add_argument("--build-only", action="store_true", help="compile the ARM64 smoke executable without requiring adb/device")
    args = parser.parse_args()
    try:
        ndk = resolve_ndk(args.ndk)
        compile_smoke(ndk)
        if args.build_only:
            return 0
        adb = find_adb(ndk)
        serial = select_device(adb, args.serial)
        run_on_device(adb, serial)
        return 0
    except (RuntimeError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
