#!/bin/bash
set -euo pipefail
project_root="$(cd "$(dirname "$0")/.." && pwd)"
adb_path="${MICRODUCK_ADB:-$HOME/Library/Android/sdk/platform-tools/adb}"
if [ ! -x "$adb_path" ]; then adb_path=adb; fi
if [ -n "${MICRODUCK_QUEST_SERIAL:-}" ]; then
  quest_args=(-s "$MICRODUCK_QUEST_SERIAL")
else
  quest_args=(-d)
fi
model="$("$adb_path" "${quest_args[@]}" shell getprop ro.product.model | tr -d '\r')"
if [[ "$model" != *Quest* ]]; then
  echo "Expected a connected Quest, found '$model'. Set MICRODUCK_QUEST_SERIAL if needed." >&2
  exit 1
fi
apk="$project_root/Builds/QuestSmokeTest/MicroDuckQuestSmokeTest.apk"
if [ ! -f "$apk" ]; then echo "Build the Quest smoke APK in Unity first." >&2; exit 1; fi
"$adb_path" "${quest_args[@]}" install -r "$apk"
"$adb_path" "${quest_args[@]}" shell am force-stop com.microduck.herding.smoketest
"$adb_path" "${quest_args[@]}" shell am start -n com.microduck.herding.smoketest/com.unity3d.player.UnityPlayerGameActivity
echo "Put on the headset. Look down toward the duck; walking starts after five simulation seconds."
echo "This is the separate smoke app. Logs below; Ctrl-C stops log capture, not the app."
quest_pid="$("$adb_path" "${quest_args[@]}" shell pidof com.microduck.herding.smoketest | tr -d '\r')"
if [ -z "$quest_pid" ]; then echo "App exited before log capture." >&2; exit 1; fi
"$adb_path" "${quest_args[@]}" logcat --pid="$quest_pid" -s Unity
