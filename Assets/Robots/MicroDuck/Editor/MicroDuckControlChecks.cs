using System;
using System.Reflection;
using Mujoco;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

// Bounded integration check of the real native libraries and imported MuJoCo
// model. The forced tip is a test input; production never changes the root pose.
public static unsafe class MicroDuckControlChecks
{
    [Serializable] sealed class Report
    {
        public bool startupReady, walked, fallObserved, recovered, clockPreserved, anchorUnchanged;
        public float forwardDistance, finalUprightness;
        public string finalState;
        public double finalSimulationTime;
        public Vector3 startFeet, endFeet, requested;
        public string motor, jointTargets;
    }

    [MenuItem("Tools/MicroDuck/Check Native Control Integration")]
    public static void RunFromMenu() => Debug.Log(Run());

    public static string Run()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
            throw new InvalidOperationException("Run the native integration check in Edit mode.");
        var scene = EditorSceneManager.NewPreviewScene();
        GameObject root = null;
        try
        {
            var floor = new GameObject("Control check floor");
            SceneManager.MoveGameObjectToScene(floor, scene);
            floor.layer = 7;
            // Keep this fixture clear of any room surfaces in the open scene.
            floor.transform.position = new Vector3(100f, 0f, 100f);
            var box = floor.AddComponent<BoxCollider>();
            box.size = new Vector3(10f, 0.1f, 10f);
            box.center = new Vector3(0f, -0.05f, 0f);
            root = (GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>(MicroDuckImporter.PrefabPath), scene);
            root.transform.position = floor.transform.position;
            var duck = root.GetComponent<MicroDuckPolicyController>();
            duck.InitializeSimulation();
            var anchor = root.transform.position;
            var rotation = root.transform.rotation;
            var report = new Report();
            for (var i = 0; i < 300; i++) duck.SimulateStep(0.02f);
            report.startupReady = duck.ControlReady;
            var start = duck.FootMidpoint;
            for (var i = 0; i < 200; i++)
            {
                duck.SetVelocityCommand(0.3f, 0f, 0f);
                duck.SimulateStep(0.02f);
            }
            report.startFeet = start;
            report.endFeet = duck.FootMidpoint;
            report.requested = (Vector3)typeof(MicroDuckPolicyController).GetField("requestedCommand", BindingFlags.Instance | BindingFlags.NonPublic).GetValue(duck);
            report.motor = typeof(MicroDuckPolicyController).GetField("motorOutput", BindingFlags.Instance | BindingFlags.NonPublic).GetValue(duck).ToString();
            report.forwardDistance = Vector3.Dot(duck.FootMidpoint - start, Vector3.forward);
            report.walked = report.forwardDistance > 0.10f;
            var dataField = typeof(MicroDuckPolicyController).GetField("data", BindingFlags.Instance | BindingFlags.NonPublic);
            var data = (MujocoLib.mjData_*)Pointer.Unbox(dataField.GetValue(duck));
            var beforeTip = data->time;
            var output = typeof(MicroDuckPolicyController).GetField("motorOutput", BindingFlags.Instance | BindingFlags.NonPublic).GetValue(duck);
            report.motor = "gain=" + output.GetType().GetField("gain").GetValue(output) + " torque=" + output.GetType().GetField("torque").GetValue(output);
            report.jointTargets = data->ctrl[0] + "," + data->ctrl[1] + "," + data->ctrl[2] + "," + data->ctrl[3] + "," + data->ctrl[4];
            // Supine at the current location, with the simulation/controller
            // clocks untouched, makes recovery reproducible on either backend.
            data->qpos[2] = Math.Max(data->qpos[2], 0.16);
            data->qpos[3] = 0;
            data->qpos[4] = 1;
            data->qpos[5] = data->qpos[6] = 0;
            report.clockPreserved = true;
            for (var i = 0; i < 1000; i++)
            {
                duck.SetVelocityCommand(0f, 0f, 0f);
                var priorTime = data->time;
                duck.SimulateStep(0.02f);
                report.clockPreserved &= Math.Abs(data->time - priorTime - 0.02) < 0.000001;
                report.fallObserved |= duck.IsFallen || duck.RecoveryActive;
                if (report.fallObserved && i > 100 && duck.ControlReady && !duck.IsFallen && duck.Uprightness > 0.8f)
                {
                    report.recovered = true;
                    break;
                }
            }
            report.finalSimulationTime = data->time;
            report.clockPreserved &= data->time > beforeTip;
            report.anchorUnchanged = root.transform.position == anchor && root.transform.rotation == rotation;
            report.finalState = duck.BehaviorState;
            report.finalUprightness = duck.Uprightness;
            var json = JsonUtility.ToJson(report, true);
            System.IO.Directory.CreateDirectory("Builds/Validation");
            System.IO.File.WriteAllText("Builds/Validation/control.json", json);
            Debug.Log(json);
            if (!report.startupReady || !report.walked || !report.fallObserved || !report.recovered || !report.clockPreserved || !report.anchorUnchanged)
                throw new InvalidOperationException("Native control integration check failed: " + json);
            return json;
        }
        finally
        {
            if (root != null) UnityEngine.Object.DestroyImmediate(root);
            EditorSceneManager.ClosePreviewScene(scene);
        }
    }
}
