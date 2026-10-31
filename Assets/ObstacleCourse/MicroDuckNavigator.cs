using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Rendering;

// Unity plans the route; only the MuJoCo policy moves the robot.
public sealed class MicroDuckNavigator : MonoBehaviour
{
    public string Status { get; private set; } = "Click a surface to walk there";
    public bool HasDestination { get; private set; }
    public Vector3 Destination { get; private set; }
    public float DistanceToGoal => Flat(Destination - trunk.position).magnitude;
    public Vector3 LastCommand { get; private set; }
    Transform trunk;
    MicroDuckPolicyController controller;
    bool wasRecovering;
    bool gaveGetUpWindow;
    float getUpUntil;
    NavMeshData data;
    NavMeshDataInstance instance;
    NavMeshQueryFilter filter;
    NavMeshPath path;
    int agentType = -1;
    Vector3[] corners;
    int corner;
    float nextPlan, progressAt, bestDistance;
    Vector3 previousCommand;
    float recoveryUntil;
    int recoveryAttempt;
    LineRenderer route, marker;
    Material lineMaterial;
    const float ArrivalRadius = 0.055f;

    public void Initialize(Transform body, MicroDuckPolicyController policy)
    {
        trunk = body;
        controller = policy;
        path = new NavMeshPath();
        var sources = new List<NavMeshBuildSource>();
        var bounds = new Bounds(trunk.position, Vector3.one);
        foreach (var box in FindObjectsByType<BoxCollider>())
        {
            if (box.gameObject.scene != gameObject.scene || box.gameObject.layer != 7 ||
                !box.enabled || box.isTrigger || !box.gameObject.activeInHierarchy) continue;
            sources.Add(new NavMeshBuildSource {
                shape = NavMeshBuildSourceShape.Box,
                transform = box.transform.localToWorldMatrix * Matrix4x4.Translate(box.center),
                size = box.size, area = 0
            });
            bounds.Encapsulate(box.bounds);
        }
        bounds.Expand(1f);
        var settings = NavMesh.CreateSettings();
        agentType = settings.agentTypeID;
        settings.agentRadius = 0.10f;
        settings.agentHeight = 0.28f;
        settings.agentClimb = 0.085f;
        settings.agentSlope = 35f;
        settings.overrideVoxelSize = true;
        settings.voxelSize = 0.012f;
        settings.overrideTileSize = true;
        settings.tileSize = 128;
        settings.minRegionArea = 0.01f;
        filter = new NavMeshQueryFilter { agentTypeID = settings.agentTypeID, areaMask = NavMesh.AllAreas };
        data = NavMeshBuilder.BuildNavMeshData(settings, sources, bounds, Vector3.zero, Quaternion.identity);
        if (data == null) { Status = "Could not build navigation mesh"; return; }
        instance = NavMesh.AddNavMeshData(data);
        lineMaterial = new Material(Shader.Find("Universal Render Pipeline/Unlit"));
        lineMaterial.color = new Color(0.2f, 1f, 0.65f);
        route = MakeLine("Planned route", 0.009f);
        marker = MakeLine("Destination", 0.012f);
        marker.loop = true;
    }

    LineRenderer MakeLine(string name, float width)
    {
        var line = new GameObject(name).AddComponent<LineRenderer>();
        line.transform.SetParent(transform, false);
        line.sharedMaterial = lineMaterial;
        line.widthMultiplier = width;
        line.positionCount = 0;
        line.shadowCastingMode = ShadowCastingMode.Off;
        line.receiveShadows = false;
        return line;
    }

    Vector3 Feet => controller.FootMidpoint;
    static Vector3 Flat(Vector3 v) { v.y = 0; return v; }

    public bool SetDestination(Vector3 point)
    {
        if (!instance.valid || !NavMesh.SamplePosition(point, out var hit, 0.12f, filter) ||
            Mathf.Abs(hit.position.y - point.y) > 0.07f)
        {
            Status = HasDestination ? "Point not walkable — keeping current goal" : "Click a walkable surface";
            return false;
        }
        Destination = hit.position;
        HasDestination = true;
        previousCommand = Vector3.zero;
        recoveryAttempt = 0;
        recoveryUntil = 0;
        bool planned = Plan();
        bestDistance = planned ? RemainingDistance() : float.PositiveInfinity;
        progressAt = Time.time;
        Status = planned ? "Walking to target" : "No route yet — goal kept, retrying";
        marker.positionCount = 33;
        for (int i = 0; i < 33; i++)
        {
            float angle = i * Mathf.PI * 2 / 33;
            marker.SetPosition(i, Destination + new Vector3(Mathf.Cos(angle) * 0.055f, 0.06f, Mathf.Sin(angle) * 0.055f));
        }
        return true;
    }

    bool Plan()
    {
        nextPlan = Time.time + 0.6f;
        if (!NavMesh.SamplePosition(Feet, out var start, 0.22f, filter) ||
            !NavMesh.CalculatePath(start.position, Destination, filter, path) ||
            path.status != NavMeshPathStatus.PathComplete)
        {
            corners = null;
            if (lineMaterial != null) lineMaterial.color = new Color(1f, 0.65f, 0.15f);
            return false;
        }
        if (lineMaterial != null) lineMaterial.color = new Color(0.2f, 1f, 0.65f);
        corners = path.corners;
        corner = corners.Length > 1 ? 1 : 0;
        nextPlan = Time.time + 0.6f;
        return corners.Length > 0;
    }

    float RemainingDistance()
    {
        if (corners == null || corners.Length == 0) return float.PositiveInfinity;
        float distance = Flat(corners[corner] - trunk.position).magnitude;
        for (int i = corner + 1; i < corners.Length; i++) distance += Vector3.Distance(corners[i-1], corners[i]);
        return distance;
    }

    // Called before the policy's FixedUpdate: feedback is the actual simulated pose.
    public Vector3 Tick(float dt)
    {
        LastCommand = Vector3.zero;
        if (!HasDestination) return Vector3.zero;
        if (controller.IsFallSequence)
        {
            gaveGetUpWindow = false;
            Status = controller.State == MicroDuckPolicyController.LocomotionState.LimpFall
                ? "Soft landing — goal kept" : "Motor pose ramp — goal kept";
            previousCommand = Vector3.zero;
            wasRecovering = true;
            progressAt = Time.time;
            return Vector3.zero;
        }
        // Give the actor a quiet opportunity to stand. Bound this pause: a robot
        // resting against an obstacle may need navigation's existing movement retries.
        if (!gaveGetUpWindow &&
            (wasRecovering || controller.State == MicroDuckPolicyController.LocomotionState.Recovering))
        {
            gaveGetUpWindow = true;
            getUpUntil = Time.time + 3f;
        }
        if (gaveGetUpWindow && controller.Uprightness < 0.85f && Time.time < getUpUntil)
        {
            Status = "Getting up — goal kept";
            previousCommand = Vector3.zero;
            wasRecovering = true;
            progressAt = Time.time;
            return Vector3.zero;
        }
        if (controller.Uprightness >= 0.85f) gaveGetUpWindow = false;
        if (wasRecovering)
        {
            wasRecovering = false;
            Resume();
            bestDistance = float.PositiveInfinity;
        }
        if (DistanceToGoal < ArrivalRadius && Mathf.Abs(Feet.y - Destination.y) < 0.09f)
        { Cancel("Arrived"); return Vector3.zero; }
        if (Time.time >= nextPlan) Plan();
        if (corners == null || corners.Length == 0)
        { Status = "Route blocked — goal kept, replanning"; previousCommand = Vector3.zero; return Vector3.zero; }
        while (corner < corners.Length - 1 && Flat(corners[corner] - trunk.position).magnitude < 0.055f) corner++;
        float remaining = RemainingDistance();
        if (remaining < bestDistance - 0.025f) { bestDistance = remaining; progressAt = Time.time; }
        if (Time.time - progressAt > 4f && Time.time >= recoveryUntil)
        {
            recoveryAttempt++;
            recoveryUntil = Time.time + 0.8f;
            progressAt = Time.time;
            Plan();
            if (corners == null || corners.Length == 0) return Vector3.zero;
        }

        // Project onto this segment, then aim ahead. Side drift changes the
        // direction to this point every tick, creating a lateral correction.
        Vector3 a = corners[Mathf.Max(0, corner - 1)], b = corners[corner];
        Vector3 segment = Flat(b - a);
        float length = segment.magnitude;
        float along = length > 0.001f ? Vector3.Dot(Flat(trunk.position - a), segment / length) : 0;
        Vector3 target = length > 0.001f ? a + (b-a) * Mathf.Clamp01((along + 0.14f) / length) : b;
        Vector3 error = Flat(target - trunk.position);
        Vector3 forward = Flat(trunk.forward).normalized;
        Vector3 right = Vector3.Cross(Vector3.up, forward);
        float angle = Vector3.SignedAngle(forward, error, Vector3.up);
        float speed = Mathf.Clamp(remaining * 0.8f, 0.16f, 0.35f);
        Vector3 velocity = error.normalized * speed;
        var desired = new Vector3(
            Mathf.Clamp(Vector3.Dot(velocity, forward), -0.15f, 0.35f),
            Mathf.Clamp(-Vector3.Dot(velocity, right), -0.20f, 0.20f),
            Mathf.Clamp(-angle * Mathf.Deg2Rad * 1.8f, -0.8f, 0.8f));
        // Pure turning can leave this policy planted. Keep lateral motion
        // toward the route while turning, rather than waiting for perfect yaw.
        if (Mathf.Abs(angle) > 65f) desired.x = 0;
        Status = "Walking to target";
        if (Time.time < recoveryUntil)
        {
            Status = $"Recovering / retry {recoveryAttempt} — goal kept";
            Vector3 recovery = recoveryAttempt % 3 == 1
                ? new Vector3(0.35f, 0, Mathf.Clamp(desired.z, -0.2f, 0.2f))
                : recoveryAttempt % 3 == 2
                    ? new Vector3(0, Mathf.Sign(-Vector3.Dot(error, right)) * 0.30f, 0)
                    : new Vector3(-0.25f, 0, 0);
            Vector3 step = (forward * recovery.x - right * recovery.y) * 0.5f;
            // Recovery moves must stay on the walkable surface, including edges.
            if (NavMesh.SamplePosition(Feet, out var origin, 0.15f, filter) &&
                !NavMesh.Raycast(origin.position, origin.position + step, out _, filter))
                desired = recovery;
        }
        previousCommand.x = Mathf.MoveTowards(previousCommand.x, desired.x, dt * 0.45f);
        previousCommand.y = Mathf.MoveTowards(previousCommand.y, desired.y, dt * 0.45f);
        previousCommand.z = Mathf.MoveTowards(previousCommand.z, desired.z, dt * 2f);
        LastCommand = previousCommand;
        route.positionCount = corners.Length - corner + 1;
        route.SetPosition(0, Feet + Vector3.up * 0.06f);
        for (int i = corner; i < corners.Length; i++) route.SetPosition(i - corner + 1, corners[i] + Vector3.up * 0.06f);
        return LastCommand;
    }

    public void Suspend()
    {
        previousCommand = LastCommand = Vector3.zero;
        if (HasDestination) Status = "Paused — goal kept";
    }

    public void Resume()
    {
        progressAt = Time.time;
        recoveryUntil = 0;
        nextPlan = 0;
    }

    public void Cancel(string message = "Stopped")
    {
        HasDestination = false;
        previousCommand = LastCommand = Vector3.zero;
        Status = message;
        if (route != null) route.positionCount = 0;
        if (marker != null) marker.positionCount = 0;
    }

    void OnDestroy()
    {
        if (instance.valid) instance.Remove();
        if (data != null) Destroy(data);
        if (agentType >= 0) NavMesh.RemoveSettings(agentType);
        path?.ClearCorners();
        if (lineMaterial != null) Destroy(lineMaterial);
    }
}
