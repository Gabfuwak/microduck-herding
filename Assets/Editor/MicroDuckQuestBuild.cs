using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.XR;

public static class MicroDuckQuestBuild
{
    const string ScenePath = "Assets/Scenes/MicroDuckQuestSmokeTest.unity";
    [MenuItem("Tools/MicroDuck/Restore Main App Identity")]
    public static void RestoreMainAppIdentity()
    {
        PlayerSettings.SetApplicationIdentifier(UnityEditor.Build.NamedBuildTarget.Android, "com.DefaultCompany.MixedRealityTemplate");
        PlayerSettings.productName = "herding";
        AssetDatabase.SaveAssets();
        Debug.Log("Restored the main app identity after the isolated smoke build.");
    }
    [MenuItem("Tools/MicroDuck/Create Quest Smoke Test")]
    public static void CreateScene()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode) throw new InvalidOperationException("Stop Play mode first.");
        if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
        var scene = EditorSceneManager.OpenScene("Assets/Scenes/MicroDuckMujocoSandbox.unity");
        UnityEngine.Object.DestroyImmediate(UnityEngine.Object.FindFirstObjectByType<MicroDuckSandboxHUD>());
        var camera = Camera.main;
        var rig = new GameObject("XR tracking space");
        rig.transform.position = new Vector3(0, 0, -1.1f);
        camera.transform.SetParent(rig.transform, false);
        camera.transform.localPosition = new Vector3(0, 1.6f, 0);
        camera.transform.localRotation = Quaternion.identity;
        var tracker = camera.gameObject.AddComponent<TrackedPoseDriver>();
        tracker.positionInput = new InputActionProperty(new InputAction("Head position", InputActionType.Value, "<XRHMD>/centerEyePosition", expectedControlType: "Vector3"));
        tracker.rotationInput = new InputActionProperty(new InputAction("Head rotation", InputActionType.Value, "<XRHMD>/centerEyeRotation", expectedControlType: "Quaternion"));
        tracker.trackingStateInput = new InputActionProperty(new InputAction("Head tracking", InputActionType.Value, "<XRHMD>/trackingState", expectedControlType: "Integer"));
        var duck = UnityEngine.Object.FindFirstObjectByType<MicroDuckMujocoController>();
        duck.transform.position = new Vector3(0, 0.8f, 0);
        duck.keyboardControl = false;
        var floor = GameObject.Find("MuJoCo floor visual");
        floor.transform.position = new Vector3(0, 0.8f, 0);
        floor.transform.localScale = new Vector3(0.3f, 1, 0.3f);
        EditorSceneManager.SaveScene(scene, ScenePath);
        Debug.Log("Quest smoke-test scene created. One duck, tracked headset camera, automatic walking and diagnostics.");
    }
    [MenuItem("Tools/MicroDuck/Build Quest Smoke Test APK")]
    public static void Build()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode) throw new InvalidOperationException("Stop Play mode first.");
        if (!File.Exists(ScenePath)) throw new InvalidOperationException("Create the Quest smoke-test scene first.");
        if (!File.Exists("Packages/org.mujoco/Plugins/Android/arm64-v8a/libmujoco.so"))
            throw new InvalidOperationException("Build the Android MuJoCo library with Tools/build_mujoco_android.sh first.");
        if (EditorUserBuildSettings.activeBuildTarget != BuildTarget.Android &&
            !EditorUserBuildSettings.SwitchActiveBuildTarget(BuildTargetGroup.Android, BuildTarget.Android))
            throw new InvalidOperationException("Could not activate the Android build target.");
        Directory.CreateDirectory("Builds/QuestSmokeTest");
        var target = UnityEditor.Build.NamedBuildTarget.Android;
        string originalId = PlayerSettings.GetApplicationIdentifier(target);
        string originalName = PlayerSettings.productName;
        BuildReport report;
        try
        {
            PlayerSettings.SetApplicationIdentifier(target, "com.microduck.herding.smoketest");
            PlayerSettings.productName = "MicroDuck Quest Smoke Test";
            report = BuildPipeline.BuildPlayer(new BuildPlayerOptions {
                scenes = new[] { ScenePath }, target = BuildTarget.Android,
                locationPathName = "Builds/QuestSmokeTest/MicroDuckQuestSmokeTest.apk",
                options = BuildOptions.Development
            });
        }
        finally
        {
            PlayerSettings.SetApplicationIdentifier(target, originalId);
            PlayerSettings.productName = originalName;
            AssetDatabase.SaveAssets();
        }
        if (report.summary.result != BuildResult.Succeeded)
            throw new InvalidOperationException($"Quest smoke build failed: {report.summary.result}, {report.summary.totalErrors} errors");
        Debug.Log("Quest smoke APK ready: Builds/QuestSmokeTest/MicroDuckQuestSmokeTest.apk");
    }
}
