using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using UnityEditor;
using UnityEngine;
using UnityEngine.AI;
using Object = UnityEngine.Object;

// Bounded checks for the integration contract, not a second navigation system.
public static class MicroDuckNavigationChecks
{
    public const string ResultPath = "Builds/Validation/navigation-checks.json";
    const BindingFlags PrivateInstance = BindingFlags.Instance | BindingFlags.NonPublic;
    [Serializable]
    sealed class Result
    {
        public bool flatTrial;
        public bool behindArrived;
        public float behindDuration;
        public float behindHeadingChange;
        public float behindFinalUprightness;
        public Vector3 behindFinalFeet;
        public bool generatedDropConnected;
        public bool reverseDropBlocked;
        public bool behindTurnBounded;
        public bool behindTurnDirectionStable;
        public bool turnsAtZeroSpeed;
        public float maximumYaw;
        public float maximumYawStep;
        public string runtime = "not run: enter the desktop trial Play mode, then run again";
        public bool rejectedGoalRetained;
        public bool anchorUnchanged = true;
        public bool fell;
        public bool goalRetainedDuringRecovery = true;
        public bool landed;
        public bool resumed;
        public bool arrived;
        public Vector3 finalFeet;
        public Vector3 finalForward;
        public Vector3 finalCommand;
        public Vector3 finalAdviserPosition;
        public Vector3 finalDesiredVelocity;
        public Vector3 finalSteeringTarget;
        public float finalUprightness;
        public string finalStatus;
        public string error;
    }

    static Result result;
    static MicroDuckNavigator navigator;
    static MicroDuckPolicyController controller;
    static Vector3 anchorPosition;
    static Quaternion anchorRotation;
    static Vector3 initialForward;
    static double beganAt;
    static float phaseBeganAt;
    static bool running;
    static int phase;

    [MenuItem("Tools/MicroDuck/Check Navigation Integration")]
    public static void RunFromMenu() => Debug.Log(Run());

    public static string Run() => Begin(false);
    public static string RunFlat() => Begin(true);

    static string Begin(bool flat)
    {
        if (running) return "Navigation runtime checks already running; results: " + ResultPath;
        result = new Result { flatTrial = flat };
        try
        {
            CheckGeneratedLinks();
            CheckBehindTurn();
            if (EditorApplication.isPlaying)
            {
                if (flat)
                {
                    var trial = Object.FindFirstObjectByType<MicroDuckDesktopTrial>();
                    if (trial == null) throw new InvalidOperationException("Open the dedicated desktop trial scene first.");
                    trial.ResetPlacement(new Vector3(-2f, 0f, -1f));
                }
                beganAt = EditorApplication.timeSinceStartup;
                phase = 0;
                running = true;
                result.runtime = "running";
                EditorApplication.update += TickRuntime;
            }
        }
        catch (Exception exception) { result.error = exception.ToString(); }
        return Save();
    }

    static void CheckGeneratedLinks()
    {
        var createSettings = typeof(MicroDuckNavigator).GetMethod("CreateBuildSettings", BindingFlags.Static | BindingFlags.NonPublic);
        var settings = (NavMeshBuildSettings)createSettings.Invoke(null, new object[] { 0.9f, 0.16f });
        var offset = new Vector3(200f, 0f, 200f);
        var sources = new List<NavMeshBuildSource>
        {
            Box(offset + new Vector3(0f, -0.05f, 0f), new Vector3(8f, 0.1f, 8f)),
            Box(offset + new Vector3(0f, 0.225f, -1f), new Vector3(1.5f, 0.45f, 1.5f))
        };
        NavMeshData data = null;
        NavMeshDataInstance instance = default;
        try
        {
            data = NavMeshBuilder.BuildNavMeshData(settings, sources, new Bounds(offset, new Vector3(10f, 3f, 10f)), Vector3.zero, Quaternion.identity);
            if (data == null) throw new InvalidOperationException("Trial navigation bake returned no data.");
            instance = NavMesh.AddNavMeshData(data);
            var filter = new NavMeshQueryFilter { agentTypeID = settings.agentTypeID, areaMask = NavMesh.AllAreas };
            var table = offset + new Vector3(0f, 0.45f, -1f);
            var floor = offset + new Vector3(0f, 0f, 2f);
            var path = new NavMeshPath();
            result.generatedDropConnected = NavMesh.CalculatePath(table, floor, filter, path) && path.status == NavMeshPathStatus.PathComplete;
            result.reverseDropBlocked = !NavMesh.CalculatePath(floor, table, filter, path) || path.status != NavMeshPathStatus.PathComplete;
        }
        finally
        {
            if (instance.valid) instance.Remove();
            if (data != null) Object.DestroyImmediate(data);
            NavMesh.RemoveSettings(settings.agentTypeID);
        }
    }

    static NavMeshBuildSource Box(Vector3 center, Vector3 size) => new()
    {
        shape = NavMeshBuildSourceShape.Box,
        transform = Matrix4x4.TRS(center, Quaternion.identity, Vector3.one),
        size = size,
        generateLinks = true
    };

    static void CheckBehindTurn()
    {
        var root = new GameObject("Navigation check — no physics");
        try
        {
            var nav = root.AddComponent<MicroDuckNavigator>();
            typeof(MicroDuckNavigator).GetField("trunk", PrivateInstance).SetValue(nav, root.transform);
            var convert = typeof(MicroDuckNavigator).GetMethod("VelocityCommand", PrivateInstance);
            var previousYaw = 0f;
            var firstSign = 0f;
            result.behindTurnBounded = result.behindTurnDirectionStable = true;
            for (var sample = 0; sample < 100; ++sample)
            {
                root.transform.rotation = Quaternion.Euler(0f, sample % 2 == 0 ? 0.01f : -0.01f, 0f);
                var command = (Vector3)convert.Invoke(nav, new object[] { Vector3.back * 0.3f, Vector3.back, 0.02f });
                result.maximumYaw = Mathf.Max(result.maximumYaw, Mathf.Abs(command.z));
                result.maximumYawStep = Mathf.Max(result.maximumYawStep, Mathf.Abs(command.z - previousYaw));
                if (firstSign == 0f && Mathf.Abs(command.z) > 0.001f) firstSign = Mathf.Sign(command.z);
                if (command.z * firstSign < 0f) result.behindTurnDirectionStable = false;
                if (Mathf.Abs(command.z) > 0.551f || Mathf.Abs(command.z - previousYaw) > 0.0161f || Mathf.Abs(command.y) > 0.121f)
                    result.behindTurnBounded = false;
                previousYaw = command.z;
            }
            result.behindTurnBounded &= result.maximumYaw > 0.1f;
            root.transform.rotation = Quaternion.identity;
            typeof(MicroDuckNavigator).GetField("previousCommand", PrivateInstance).SetValue(nav, Vector3.zero);
            var stationary = (Vector3)convert.Invoke(nav, new object[] { Vector3.zero, Vector3.back, 0.02f });
            result.turnsAtZeroSpeed = stationary.x == 0f && stationary.y == 0f &&
                Mathf.Abs(stationary.z) > 0.001f && Mathf.Abs(stationary.z) <= 0.0161f;
        }
        finally { Object.DestroyImmediate(root); }
    }

    static void TickRuntime()
    {
        try
        {
            if (!EditorApplication.isPlaying || EditorApplication.timeSinceStartup - beganAt > 45.0)
            {
                Finish("incomplete: stopped or exceeded 45 seconds wall time");
                return;
            }
            if (phase == 0)
            {
                controller = Object.FindFirstObjectByType<MicroDuckPolicyController>();
                navigator = Object.FindFirstObjectByType<MicroDuckNavigator>();
                if (controller == null || navigator == null || !controller.ControlReady ||
                    controller.GetComponentInChildren<NavMeshAgent>() is not { isOnNavMesh: true }) return;
                anchorPosition = controller.transform.position;
                anchorRotation = controller.transform.rotation;
                initialForward = Vector3.ProjectOnPlane(controller.Trunk.forward, Vector3.up).normalized;
                var behind = controller.FootMidpoint - Vector3.ProjectOnPlane(controller.Trunk.forward, Vector3.up).normalized * 0.45f;
                var agent = controller.GetComponentInChildren<NavMeshAgent>();
                var filter = new NavMeshQueryFilter { agentTypeID = agent.agentTypeID, areaMask = NavMesh.AllAreas };
                if (NavMesh.SamplePosition(behind, out var hit, 0.2f, filter)) navigator.SetDestination(hit.position);
                var accepted = navigator.Destination;
                var rejected = !navigator.SetDestination(new Vector3(500f, 100f, 500f));
                result.rejectedGoalRetained = rejected && navigator.HasDestination && Vector3.Distance(accepted, navigator.Destination) < 0.0001f;
                phaseBeganAt = Time.time;
                phase = 1;
                return;
            }
            result.anchorUnchanged &= Vector3.Distance(anchorPosition, controller.transform.position) < 0.00001f &&
                Quaternion.Angle(anchorRotation, controller.transform.rotation) < 0.001f;
            if (controller.RecoveryActive)
            {
                result.fell = true;
                result.goalRetainedDuringRecovery &= navigator.HasDestination;
            }
            if (phase == 1 && (result.flatTrial
                ? navigator.Status == "Arrived" || Time.time - phaseBeganAt > 30f
                : Time.time - phaseBeganAt > 2f))
            {
                result.behindArrived = navigator.Status == "Arrived";
                result.behindDuration = Time.time - phaseBeganAt;
                result.behindHeadingChange = Vector3.Angle(initialForward,
                    Vector3.ProjectOnPlane(controller.Trunk.forward, Vector3.up));
                result.behindFinalUprightness = controller.Uprightness;
                result.behindFinalFeet = controller.FootMidpoint;
                navigator.SetDestination(result.flatTrial ? new Vector3(-2f, 0f, 2f) : new Vector3(0f, 0f, 2f));
                phaseBeganAt = Time.time;
                phase = 2;
            }
            if (phase != 2) return;
            result.landed |= controller.FootMidpoint.y < 0.13f;
            result.resumed |= result.landed && navigator.HasDestination && navigator.LastCommand.sqrMagnitude > 0.001f;
            result.arrived = navigator.Status == "Arrived";
            result.finalStatus = navigator.Status + " / " + controller.BehaviorState;
            result.finalFeet = controller.FootMidpoint;
            result.finalForward = controller.Trunk.forward;
            result.finalCommand = navigator.LastCommand;
            result.finalUprightness = controller.Uprightness;
            var adviser = controller.GetComponentInChildren<NavMeshAgent>();
            if (adviser != null && adviser.isOnNavMesh)
            {
                result.finalAdviserPosition = adviser.nextPosition;
                result.finalDesiredVelocity = adviser.desiredVelocity;
                result.finalSteeringTarget = adviser.steeringTarget;
            }
            if (result.arrived) Finish("completed");
            else if (Time.time - phaseBeganAt > 100f) Finish("bounded trial ended before arrival");
        }
        catch (Exception exception) { result.error = exception.ToString(); Finish("failed"); }
    }

    static void Finish(string status)
    {
        EditorApplication.update -= TickRuntime;
        running = false;
        result.runtime = status;
        Debug.Log("Navigation checks: " + Save());
    }

    static string Save()
    {
        Directory.CreateDirectory(Path.GetDirectoryName(ResultPath));
        var json = JsonUtility.ToJson(result, true);
        File.WriteAllText(ResultPath, json);
        return json;
    }
}
