using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Rendering;
using UnityEngine.XR;

// Unity plans the route; the joystick remains the only continuous command owner.
[DisallowMultipleComponent]
public sealed class MicroDuckNavigator : MonoBehaviour
{
    public string Status { get; private set; } = "Click a surface to walk there";
    public bool HasDestination { get; private set; }
    public bool IsFollowing => following && HasDestination;
    public Vector3 Destination { get; private set; }
    public Vector3 LastCommand { get; private set; }

    const float ArrivalRadius = 0.055f;
    const float GoalHeightTolerance = 0.07f;
    const float FollowDistance = 0.45f;
    static readonly Color RouteColor = new(0.2f, 1f, 0.65f);
    static readonly Color UnreachableColor = new(1f, 0.65f, 0.15f);

    readonly List<NavMeshBuildSource> sources = new();
    Vector3[] cornerBuffer = new Vector3[128];
    Transform trunk;
    MicroDuckPolicyController controller;
    NavMeshData data;
    NavMeshDataInstance instance;
    NavMeshQueryFilter filter;
    NavMeshPath path;
    AsyncOperation bakeOperation;
    LineRenderer route;
    LineRenderer marker;
    Material lineMaterial;
    int agentType = -1;
    int cornerCount;
    int corner;
    int recoveryAttempt;
    float nextPlan;
    float progressAt;
    float bestDistance;
    float recoveryUntil;
    Vector3 previousCommand;
    Vector3 pendingPoint;
    bool hasPendingPoint;
    bool initialized;
    bool bakeReady;
    bool destroying;
    Transform followTarget;
    bool following;
    bool followNeedsPlan;
    Vector3 projectedFollowTarget;

    public void Initialize(MicroDuckPolicyController policy)
    {
        if (initialized)
            throw new System.InvalidOperationException("MicroDuckNavigator may only be initialized once.");
        if (policy == null)
            throw new System.ArgumentNullException(nameof(policy));

        controller = policy;
        trunk = policy.Trunk;
        path = new NavMeshPath();
        controller.GetNavigationBuildSources(sources, out var bounds);
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
        filter = new NavMeshQueryFilter { agentTypeID = agentType, areaMask = NavMesh.AllAreas };
        data = new NavMeshData(agentType);
        CreateVisuals();
        initialized = true;
        bakeOperation = NavMeshBuilder.UpdateNavMeshDataAsync(data, settings, sources, bounds);
        if (bakeOperation == null)
        {
            bakeReady = true;
            Status = "Could not start navigation mesh bake";
            return;
        }
        bakeOperation.completed += OnBakeCompleted;
    }

    void CreateVisuals()
    {
        // URP/Lit is already used by the included MicroDuck materials, so the
        // shader is retained by the player build rather than relying on an
        // otherwise-unreferenced Shader.Find-only shader.
        var shader = Shader.Find("Universal Render Pipeline/Lit");
        if (shader == null)
            return;
        lineMaterial = new Material(shader);
        SetVisualColor(RouteColor);
        route = MakeLine("Planned route", 0.009f);
        marker = MakeLine("Destination", 0.012f);
        marker.loop = true;
    }

    LineRenderer MakeLine(string objectName, float width)
    {
        var lineObject = new GameObject(objectName);
        lineObject.transform.SetParent(transform, false);
        var renderer = lineObject.AddComponent<LineRenderer>();
        renderer.sharedMaterial = lineMaterial;
        renderer.widthMultiplier = width;
        renderer.positionCount = 0;
        renderer.generateLightingData = true;
        renderer.shadowCastingMode = ShadowCastingMode.Off;
        renderer.receiveShadows = false;
        return renderer;
    }

    public bool SetDestination(Vector3 worldPoint)
    {
        if (!initialized || destroying)
            return false;
        if (!bakeReady)
        {
            pendingPoint = worldPoint;
            hasPendingPoint = true;
            HasDestination = true;
            following = false;
            followNeedsPlan = false;
            followTarget = null;
            return true;
        }
        return AcceptDestination(worldPoint);
    }

    bool AcceptDestination(Vector3 worldPoint)
    {
        if (!instance.valid || !NavMesh.SamplePosition(worldPoint, out var hit, 0.12f, filter) ||
            Mathf.Abs(hit.position.y - worldPoint.y) > GoalHeightTolerance)
        {
            Status = HasDestination ? "Point not walkable — keeping current goal" : "Click a walkable surface";
            return false;
        }

        Destination = hit.position;
        HasDestination = true;
        following = false;
        followNeedsPlan = false;
        followTarget = null;
        previousCommand = Vector3.zero;
        recoveryAttempt = 0;
        recoveryUntil = 0f;
        progressAt = Time.time;
        var planned = Plan();
        bestDistance = planned ? RemainingDistance() : float.PositiveInfinity;
        Status = planned ? "Walking to target" : "No route yet — goal kept, retrying";
        DrawMarker();
        return true;
    }

    public bool Follow(Transform target)
    {
        if (!initialized || destroying || target == null)
            return false;

        followTarget = target;
        following = true;
        followNeedsPlan = true;
        hasPendingPoint = false;
        HasDestination = true;
        nextPlan = 0f;
        previousCommand = LastCommand = Vector3.zero;
        progressAt = Time.time;
        recoveryUntil = 0f;
        cornerCount = 0;
        Status = "Following headset";
        return true;
    }

    void OnBakeCompleted(AsyncOperation operation)
    {
        if (destroying)
        {
            ReleaseResources();
            return;
        }
        if (data == null)
            return;
        bakeReady = true;
        if (sources.Count == 0)
        {
            Status = "Could not build navigation mesh";
            HasDestination = false;
            hasPendingPoint = false;
            return;
        }
        instance = NavMesh.AddNavMeshData(data);
        if (!instance.valid)
        {
            Status = "Could not add navigation mesh";
            HasDestination = false;
            hasPendingPoint = false;
            return;
        }
        if (following)
        {
            nextPlan = 0f;
            return;
        }
        if (hasPendingPoint)
        {
            var point = pendingPoint;
            hasPendingPoint = false;
            if (!AcceptDestination(point))
                HasDestination = false;
        }
    }

    bool Plan()
    {
        nextPlan = Time.time + 0.6f;
        cornerCount = 0;
        if (!instance.valid || !NavMesh.SamplePosition(controller.FootMidpoint, out var start, 0.22f, filter) ||
            !NavMesh.CalculatePath(start.position, Destination, filter, path) ||
            path.status != NavMeshPathStatus.PathComplete)
        {
            SetVisualColor(UnreachableColor);
            if (route != null) route.positionCount = 0;
            return false;
        }
        cornerCount = path.GetCornersNonAlloc(cornerBuffer);
        while (cornerCount >= cornerBuffer.Length)
        {
            System.Array.Resize(ref cornerBuffer, Mathf.Max(cornerCount + 1, cornerBuffer.Length * 2));
            cornerCount = path.GetCornersNonAlloc(cornerBuffer);
        }
        if (cornerCount <= 0)
        {
            SetVisualColor(UnreachableColor);
            if (route != null) route.positionCount = 0;
            return false;
        }
        corner = cornerCount > 1 ? 1 : 0;
        SetVisualColor(RouteColor);
        nextPlan = Time.time + 0.6f;
        return true;
    }

    float RemainingDistance()
    {
        if (cornerCount == 0)
            return float.PositiveInfinity;
        var distance = Flat(cornerBuffer[corner] - trunk.position).magnitude;
        for (var i = corner + 1; i < cornerCount; i++)
            distance += Vector3.Distance(cornerBuffer[i - 1], cornerBuffer[i]);
        return distance;
    }

    bool TryGetFollowGoal(Vector3 feet, out (Vector3 target, Vector3 destination) goal)
    {
        goal = default;
        if (followTarget == null || !HeadsetTrackingAvailable())
            return false;

        // Project vertically onto the duck's walking level, not onto an
        // arbitrary nearby table. Keep the headset's exact XZ as the centre.
        var headPosition = followTarget.position;
        headPosition.y = feet.y;
        if (!NavMesh.SamplePosition(headPosition, out var surface, 0.16f, filter) ||
            Mathf.Abs(surface.position.y - feet.y) > GoalHeightTolerance)
            return false;
        headPosition.y = surface.position.y;
        var towardDuck = Flat(trunk.position - headPosition);
        var destination = headPosition + towardDuck.normalized * FollowDistance;
        if (!NavMesh.SamplePosition(destination, out var approach, 0.12f, filter) ||
            Mathf.Abs(approach.position.y - headPosition.y) > GoalHeightTolerance ||
            Flat(approach.position - headPosition).sqrMagnitude < FollowDistance * FollowDistance - 0.0001f)
            return false;
        goal = (headPosition, approach.position);
        return true;
    }

    static bool HeadsetTrackingAvailable()
    {
        var device = InputDevices.GetDeviceAtXRNode(XRNode.CenterEye);
        if (device.isValid && device.TryGetFeatureValue(CommonUsages.isTracked, out var tracked))
            return tracked;
#if UNITY_EDITOR
        return true;
#else
        return false;
#endif
    }

    public Vector3 Tick(float dt)
    {
        LastCommand = Vector3.zero;
        if (!initialized || !bakeReady || !instance.valid || !HasDestination)
            return Vector3.zero;
        var feet = controller.FootMidpoint;
        if (following)
        {
            if (!TryGetFollowGoal(feet, out var goal))
            {
                followNeedsPlan = true;
                cornerCount = 0;
                if (route != null) route.positionCount = 0;
                if (marker != null) marker.positionCount = 0;
                previousCommand = LastCommand = Vector3.zero;
                Status = "Headset unavailable — waiting";
                return Vector3.zero;
            }

            projectedFollowTarget = goal.target;
            if (Time.time >= nextPlan &&
                (followNeedsPlan || Flat(Destination - goal.destination).sqrMagnitude > 0.0064f ||
                 cornerCount == 0))
            {
                Destination = goal.destination;
                var planned = Plan();
                followNeedsPlan = false;
                bestDistance = planned ? RemainingDistance() : float.PositiveInfinity;
                if (marker != null) DrawMarker();
                Status = planned ? "Following headset" : "No route yet — following, retrying";
            }
            var toHeadset = Flat(goal.target - trunk.position);
            if (toHeadset.sqrMagnitude <= FollowDistance * FollowDistance)
            {
                previousCommand = LastCommand = Vector3.zero;
                Status = "Following — holding distance";
                return Vector3.zero;
            }
        }
        else if (Flat(Destination - trunk.position).magnitude < ArrivalRadius &&
                 Mathf.Abs(feet.y - Destination.y) < 0.09f)
        {
            Cancel();
            Status = "Arrived";
            return Vector3.zero;
        }
        if (Time.time >= nextPlan)
            Plan();
        if (cornerCount == 0)
        {
            Status = "Route blocked — goal kept, replanning";
            previousCommand = Vector3.zero;
            return Vector3.zero;
        }
        while (corner < cornerCount - 1 && Flat(cornerBuffer[corner] - trunk.position).magnitude < 0.055f)
            corner++;
        var remaining = RemainingDistance();
        if (remaining < bestDistance - 0.025f)
        {
            bestDistance = remaining;
            progressAt = Time.time;
        }
        if (!following && Time.time - progressAt > 4f && Time.time >= recoveryUntil)
        {
            recoveryAttempt++;
            recoveryUntil = Time.time + 0.8f;
            progressAt = Time.time;
            Plan();
            if (cornerCount == 0)
                return Vector3.zero;
        }

        var a = cornerBuffer[Mathf.Max(0, corner - 1)];
        var b = cornerBuffer[corner];
        var segment = Flat(b - a);
        var length = segment.magnitude;
        var along = length > 0.001f ? Vector3.Dot(Flat(trunk.position - a), segment / length) : 0f;
        var target = length > 0.001f ? a + (b - a) * Mathf.Clamp01((along + 0.14f) / length) : b;
        var error = Flat(target - trunk.position);
        var forward = Flat(trunk.forward);
        if (forward.sqrMagnitude < 0.000001f)
        {
            previousCommand = Vector3.zero;
            return Vector3.zero;
        }
        forward.Normalize();
        var right = Vector3.Cross(Vector3.up, forward);
        var angle = error.sqrMagnitude > 0.000001f ? Vector3.SignedAngle(forward, error, Vector3.up) : 0f;
        var speed = Mathf.Clamp(remaining * 0.8f, 0.16f, 0.35f);
        if (following)
            speed = Mathf.Min(speed, Mathf.Max(0f, Flat(projectedFollowTarget - trunk.position).magnitude - FollowDistance) * 1.2f);
        var velocity = error.sqrMagnitude > 0.000001f ? error.normalized * speed : Vector3.zero;
        var desired = new Vector3(
            Mathf.Clamp(Vector3.Dot(velocity, forward), -0.15f, 0.35f),
            Mathf.Clamp(-Vector3.Dot(velocity, right), -0.20f, 0.20f),
            Mathf.Clamp(-angle * Mathf.Deg2Rad * 1.8f, -0.8f, 0.8f));
        if (Mathf.Abs(angle) > 65f)
            desired.x = 0f;
        Status = following ? "Following headset" : "Walking to target";
        if (!following && Time.time < recoveryUntil)
        {
            Status = "Retrying route — goal kept";
            var recovery = recoveryAttempt % 3 == 1
                ? new Vector3(0.35f, 0f, Mathf.Clamp(desired.z, -0.2f, 0.2f))
                : recoveryAttempt % 3 == 2
                    ? new Vector3(0f, Mathf.Sign(-Vector3.Dot(error, right)) * 0.30f, 0f)
                    : new Vector3(-0.25f, 0f, 0f);
            var step = (forward * recovery.x - right * recovery.y) * 0.5f;
            if (NavMesh.SamplePosition(feet, out var origin, 0.15f, filter) &&
                !NavMesh.Raycast(origin.position, origin.position + step, out _, filter))
                desired = recovery;
        }
        previousCommand.x = Mathf.MoveTowards(previousCommand.x, desired.x, dt * 0.45f);
        previousCommand.y = Mathf.MoveTowards(previousCommand.y, desired.y, dt * 0.45f);
        previousCommand.z = Mathf.MoveTowards(previousCommand.z, desired.z, dt * 2f);
        if (following)
        {
            var toHeadset = Flat(projectedFollowTarget - trunk.position);
            var distance = toHeadset.magnitude;
            var inward = toHeadset / distance;
            var worldCommand = forward * previousCommand.x - right * previousCommand.y;
            var closingSpeed = Vector3.Dot(worldCommand, inward);
            var maxClosingSpeed = Mathf.Max(0f, distance - FollowDistance) / Mathf.Max(dt, 0.0001f);
            if (closingSpeed > maxClosingSpeed)
            {
                worldCommand -= inward * (closingSpeed - maxClosingSpeed);
                previousCommand.x = Vector3.Dot(worldCommand, forward);
                previousCommand.y = -Vector3.Dot(worldCommand, right);
            }
        }
        LastCommand = previousCommand;
        DrawRoute(feet);
        return LastCommand;
    }

    void DrawRoute(Vector3 feet)
    {
        if (route == null)
            return;
        route.positionCount = cornerCount - corner + 1;
        route.SetPosition(0, feet + Vector3.up * 0.06f);
        for (var i = corner; i < cornerCount; i++)
            route.SetPosition(i - corner + 1, cornerBuffer[i] + Vector3.up * 0.06f);
    }

    void DrawMarker()
    {
        if (marker == null)
            return;
        marker.positionCount = 33;
        for (var i = 0; i < 33; i++)
        {
            var angle = i * Mathf.PI * 2f / 33f;
            marker.SetPosition(i, Destination + new Vector3(Mathf.Cos(angle) * 0.055f, 0.06f,
                Mathf.Sin(angle) * 0.055f));
        }
    }

    void SetVisualColor(Color color)
    {
        if (lineMaterial != null)
            lineMaterial.SetColor("_BaseColor", color);
    }

    static Vector3 Flat(Vector3 value)
    {
        value.y = 0f;
        return value;
    }

    public void Suspend()
    {
        previousCommand = LastCommand = Vector3.zero;
        if (HasDestination)
            Status = "Paused — goal kept";
    }

    public void Resume()
    {
        progressAt = Time.time;
        recoveryUntil = 0f;
        nextPlan = 0f;
    }

    public void Cancel()
    {
        HasDestination = false;
        hasPendingPoint = false;
        following = false;
        followNeedsPlan = false;
        followTarget = null;
        previousCommand = LastCommand = Vector3.zero;
        Status = "Stopped";
        if (route != null) route.positionCount = 0;
        if (marker != null) marker.positionCount = 0;
    }

    void OnDestroy()
    {
        destroying = true;
        if (bakeOperation != null && !bakeOperation.isDone)
        {
            return;
        }
        ReleaseResources();
    }

    void ReleaseResources()
    {
        if (instance.valid)
            instance.Remove();
        if (data != null)
        {
            Destroy(data);
            data = null;
        }
        if (agentType >= 0)
        {
            NavMesh.RemoveSettings(agentType);
            agentType = -1;
        }
        if (lineMaterial != null)
        {
            Destroy(lineMaterial);
            lineMaterial = null;
        }
        bakeOperation = null;
    }
}
