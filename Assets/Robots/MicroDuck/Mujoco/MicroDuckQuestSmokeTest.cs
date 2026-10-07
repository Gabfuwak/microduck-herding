using System;
using System.IO;
using System.Collections.Generic;
using UnityEngine.XR;
using UnityEngine;
using UnityEngine.SceneManagement;

// Runs in the dedicated smoke-test scene and records evidence in app-private
// storage. A desktop pass does not count as an Android/Quest pass.
public sealed class MicroDuckQuestSmokeTest : MonoBehaviour
{
    MicroDuckMujocoController duck;
    TextMesh status;
    float began, nextReport, reportBegan;
    int frames;
    bool completed;
    Vector3 startPosition;
    bool hadStart, trackingConfigured;
    readonly List<XRInputSubsystem> xrInputs = new List<XRInputSubsystem>();
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Install()
    {
        if (SceneManager.GetActiveScene().name != "MicroDuckQuestSmokeTest") return;
        new GameObject("Quest smoke-test diagnostics").AddComponent<MicroDuckQuestSmokeTest>();
    }
    void Start()
    {
        duck = FindFirstObjectByType<MicroDuckMujocoController>();
        began = reportBegan = Time.realtimeSinceStartup;
        File.WriteAllText(Path.Combine(Application.persistentDataPath, "microduck-smoke-result.json"), "{\"status\":\"running\"}");
        nextReport = began + 5;
        var label = new GameObject("Smoke test status");
        label.transform.position = new Vector3(0, 1.35f, 0.35f);
        label.transform.rotation = Quaternion.identity;
        status = label.AddComponent<TextMesh>();
        status.anchor = TextAnchor.MiddleCenter;
        status.alignment = TextAlignment.Center;
        status.characterSize = 0.012f;
        status.fontSize = 48;
        status.text = "MicroDuck: loading native physics and policy…";
        Debug.Log($"MicroDuck smoke: {Application.platform}, {SystemInfo.deviceModel}, {SystemInfo.processorType}, Unity {Application.unityVersion}");
    }
    void Update()
    {
        frames++;
        if (!trackingConfigured)
        {
            SubsystemManager.GetSubsystems(xrInputs);
            foreach (var input in xrInputs)
            {
                if (!input.running) continue;
                bool floor = input.TrySetTrackingOriginMode(TrackingOriginModeFlags.Floor);
                var rig = Camera.main == null ? null : Camera.main.transform.parent;
                if (rig != null)
                {
                    Vector3 position = rig.position;
                    position.y = floor ? 0 : 1.6f;
                    rig.position = position;
                }
                trackingConfigured = true;
                Debug.Log($"MicroDuck smoke: XR origin {input.GetTrackingOriginMode()}");
                break;
            }
        }
        if (duck == null) { status.text = "FAIL: no duck in smoke scene"; return; }
        if (duck.InitializationError != null)
        {
            status.text = "FAIL: " + duck.InitializationError;
            if (!completed) Record(false, duck.InitializationError);
            return;
        }
        if (!duck.IsReady)
        {
            if (!completed && Time.realtimeSinceStartup - began > 60) Record(false, "Initialization timed out");
            return;
        }
        if (!hadStart) { startPosition = duck.links[0].position; hadStart = true; }
        float now = Time.realtimeSinceStartup;
        if (now < nextReport) return;
        float fps = frames / (now - reportBegan);
        frames = 0; reportBegan = now; nextReport = now + 5;
        float distance = Vector3.Distance(startPosition, duck.links[0].position);
        status.text = $"MuJoCo + ONNX: {duck.SimulationTime:F1}s\n{fps:F0} FPS · upright {duck.IsUpright} · recoveries {duck.RecoveryCount}\nMovement {distance:F2} m";
        Debug.Log($"MicroDuck smoke: wall={now-began:F1}s, sim={duck.SimulationTime:F1}s, fps={fps:F1}, upright={duck.IsUpright}, recoveries={duck.RecoveryCount}, displacement={distance:F3}m");
        if (!completed && duck.SimulationTime >= 30)
        {
            bool passed = duck.IsUpright && duck.RecoveryCount == 0 && distance > 0.05f;
            Record(passed, passed ? "Native loading, model, policy, and 30s walking passed" : "Walking/upright criteria failed");
            status.text += passed ? "\nPASS: 30s walking" : "\nFAIL: walking criteria";
        }
    }
    void Record(bool passed, string message)
    {
        completed = true;
        var result = new Result {
            passed = passed, message = message, platform = Application.platform.ToString(),
            device = SystemInfo.deviceModel, unity = Application.unityVersion,
            simulationSeconds = duck == null ? 0 : duck.SimulationTime,
            wallSeconds = Time.realtimeSinceStartup - began,
            recoveries = duck == null ? 0 : duck.RecoveryCount
        };
        string json = JsonUtility.ToJson(result, true);
        File.WriteAllText(Path.Combine(Application.persistentDataPath, "microduck-smoke-result.json"), json);
        Debug.Log("MicroDuck smoke result: " + json);
    }
    [Serializable] class Result
    {
        public bool passed;
        public string message, platform, device, unity;
        public double simulationSeconds;
        public float wallSeconds;
        public int recoveries;
    }
}
