# Quest native MuJoCo smoke test

This isolates native MuJoCo loading and the pretrained walking policy from room
scanning, placement, hand input and shared collisions. The scene has one duck on
a horizontal surface at 0.8 m and a tracked headset camera. Look down and forward
toward the duck. Walking starts after five simulation seconds.

1. Install Unity 6000.6.3f1 Android Build Support, OpenJDK, SDK and NDK.
2. Run `Tools/build_mujoco_android.sh`. This builds pinned upstream MuJoCo 3.15.0
   with Android ARM64, API 34, static libc++, and 16 KiB linker page alignment.
3. In Unity, Tools > MicroDuck > Create Quest Smoke Test.
4. Tools > MicroDuck > Build Quest Smoke Test APK. The existing project targets
   Android ARM64 and IL2CPP. This build uses only the smoke scene.
5. With a Quest connected and USB debugging authorized, run
   `Tools/run_quest_smoke_test.sh` to install, launch, and watch the app logs.
   Or run the equivalent commands:

```sh
adb devices -l
adb install -r Builds/QuestSmokeTest/MicroDuckQuestSmokeTest.apk
adb shell am start -n com.microduck.herding.smoketest/com.unity3d.player.UnityPlayerGameActivity
adb logcat -s Unity
```

The smoke APK uses its own application ID, `com.microduck.herding.smoketest`,
and does not replace the main app. The build restores the original application
ID and product name afterwards. Outputs live in ignored `Builds/QuestSmokeTest`.

The log prints model initialization, extraction, simulation time, measured frame
rate, upright state, recovery count and displacement. After 30 simulated seconds,
a pass requires the duck to be upright with zero recoveries and more than 5 cm
movement. Frame rate is reported separately; a walking pass alone does not mean
that the full herding experience meets its frame budget. A result JSON is written
to `Application.persistentDataPath/microduck-smoke-result.json`, reset to "running"
on every launch. Failure to initialize in 60 wall seconds produces a failure.

Android extracts MJCF and its nine collision STLs from the APK into private
persistent storage using UnityWebRequest. Files are refreshed each launch to
avoid reusing an old model following an APK update. No network service is needed.

## Status

MuJoCo 3.15.0 built successfully from unmodified upstream source with Unity's
NDK r27c. The stripped Android ARM64 library is 5.4 MiB and depends only on
libdl, libm and libc. SHA-256:
`bdae260149c80f93f7ec4668a3854ed1b1abc5a6d5d7d1b92c20491341132866`.

On the connected Quest 3S (Android API 34), a native C probe loaded the prepared
MJCF (nq=21, nu=14) and ran 2,000 finite physics steps: 10 simulated seconds in
0.142 wall seconds. This uses fixed joint targets, so it does not validate
walking or Unity interop. Source: `Tools/test_mujoco_android_native.c`.

The Unity Android ARM64 development APK built successfully with IL2CPP. The
final build includes the headset floor-origin adjustment. Package inspection
confirmed the exact native library and all ten runtime MJCF/mesh assets.
`Builds/QuestSmokeTest/build-validation.json` records checksums and status.
The original main app ID and product name were restored after the build.

The Quest was disconnected before APK installation. The full Unity native
interop / ONNX / walking / frame-rate headset run remains unverified. Reconnect
and run `Tools/run_quest_smoke_test.sh` to perform it. The earlier native probe
is not counted as a Unity walking pass.
