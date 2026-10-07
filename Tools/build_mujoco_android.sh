#!/bin/bash
set -euo pipefail
# Build precisely the version used by the bundled C# bindings. No downloaded
# third-party Android binary is substituted for this source build.
project_root="$(cd "$(dirname "$0")/.." && pwd)"
ndk_path="${MICRODUCK_ANDROID_NDK:-/Applications/Unity/Hub/Editor/6000.6.3f1/PlaybackEngines/AndroidPlayer/NDK}"
build_root="${MICRODUCK_ANDROID_BUILD:-/tmp/microduck-mujoco-android}"
if [ ! -f "$ndk_path/build/cmake/android.toolchain.cmake" ]; then
  echo "Missing Android NDK: install Unity 6000.6.3f1 Android SDK & NDK Tools, or set MICRODUCK_ANDROID_NDK." >&2
  exit 1
fi
if [ ! -d "$build_root/source/.git" ]; then
  git clone --branch 3.15.0 --depth 1 https://github.com/google-deepmind/mujoco.git "$build_root/source"
fi
revision="$(git -C "$build_root/source" rev-parse HEAD)"
if [ "$revision" != 9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5 ]; then
  echo "Unexpected MuJoCo source revision: $revision" >&2
  exit 1
fi
cmake -S "$build_root/source" -B "$build_root/build" -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="$ndk_path/build/cmake/android.toolchain.cmake" \
  -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-34 \
  -DANDROID_STL=c++_static -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_SHARED_LINKER_FLAGS=-Wl,-z,max-page-size=16384 \
  -DMUJOCO_BUILD_SIMULATE=OFF -DMUJOCO_BUILD_EXAMPLES=OFF \
  -DMUJOCO_BUILD_TESTS=OFF -DMUJOCO_TEST_PYTHON_UTIL=OFF \
  -DMUJOCO_ENABLE_AVX=OFF -DMUJOCO_ENABLE_AVX_INTRINSICS=OFF
cmake --build "$build_root/build" --target mujoco -j 6
output="$project_root/Packages/org.mujoco/Plugins/Android/arm64-v8a"
mkdir -p "$output"
cp "$build_root/build/lib/libmujoco.so" "$output/libmujoco.so"
"$ndk_path/toolchains/llvm/prebuilt/darwin-x86_64/bin/llvm-strip" --strip-unneeded "$output/libmujoco.so"
cat > "$output/libmujoco.so.meta" <<'META'
fileFormatVersion: 2
guid: f9c1591f3cb34b2396f7d343cf179ae2
PluginImporter:
  externalObjects: {}
  serializedVersion: 2
  iconMap: {}
  executionOrder: {}
  defineConstraints: []
  isPreloaded: 0
  isOverridable: 1
  isExplicitlyReferenced: 0
  validateReferences: 1
  platformData:
  - first:
      Any: 
    second:
      enabled: 0
      settings: {}
  - first:
      Android: Android
    second:
      enabled: 1
      settings:
        AndroidSharedLibraryType: Shared
        CPU: ARM64
  - first:
      Editor: Editor
    second:
      enabled: 0
      settings:
        CPU: AnyCPU
        DefaultValueInitialized: true
        OS: AnyOS
  userData: 
  assetBundleName: 
  assetBundleVariant: 
META
file "$output/libmujoco.so"
shasum -a 256 "$output/libmujoco.so"
echo "Android ARM64 MuJoCo installed: $output/libmujoco.so"
