using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Rendering;

// Unity's agent advises; measured MuJoCo state remains authoritative. The
// joystick is still the only continuous owner of SetVelocityCommand.
[DisallowMultipleComponent]
public sealed class MicroDuckNavigator : MonoBehaviour
{
    public string Status { get; private set; } = "Click a surface to walk there";
    public bool HasDestination { get; private set; }
    public Vector3 Destination { get; private set; }
    public Vector3 LastCommand { get; private set; }

    [SerializeField, Min(0f)] float maximumDropHeight = 0.9f;
    [SerializeField, Range(0.05f, 0.4f)] float speed = 0.40f;
    [SerializeField, Range(0.10f, 0.25f)] float agentRadius = 0.16f;
    [SerializeField, Range(0.1f, 1f)] float yawSpeed = 0.55f;

    const float ArrivalRadius = 0.065f;
    const int SurfaceLayers = (1 << 7) | (1 << 30);
    static readonly Color RouteColor = new(0.2f, 1f, 0.65f);
    static readonly Color WaitingColor = new(1f, 0.65f, 0.15f);

    readonly List<NavMeshBuildSource> sources = new();
    MicroDuckPolicyController controller;
    Transform trunk;
    NavMeshAgent agent;
    NavMeshData data;
    NavMeshDataInstance instance;
    NavMeshQueryFilter filter;
    AsyncOperation bakeOperation;
    LineRenderer route;
    LineRenderer marker;
    Material lineMaterial;
    Vector3[] corners = System.Array.Empty<Vector3>();
    Vector3 pendingPoint;
    Vector3 previousCommand;
    Vector3 previousFeet;
    Vector3 dropStart;
    Vector3 dropEnd;
    float nextPlan;
    float nextRouteDraw;
    float bestDistance = float.PositiveInfinity;
    float progressAt;
    float quietUntil;
    float landingSince = -1f;
    float dropStarted;
    float turnSign = 1f;
    int agentType = -1;
    bool initialized;
    bool bakeReady;
    bool hasPendingPoint;
    bool recovering;
    bool previousFallen;
    bool crossingDrop;
    bool leftPlatform;
    bool destroying;

    public void Initialize(MicroDuckPolicyController policy)
    {
        if (initialized) throw new System.InvalidOperationException("MicroDuckNavigator may only be initialized once.");
        if (policy == null) throw new System.ArgumentNullException(nameof(policy));
        if (!policy.SimulationReady) throw new System.InvalidOperationException("Initialize the simulation before navigation.");
        controller = policy;
        trunk = policy.Trunk;
        previousFeet = policy.FootMidpoint;
        controller.GetNavigationBuildSources(sources, out var bounds);
        for (var i = 0; i < sources.Count; ++i)
        {
            var source = sources[i];
            source.generateLinks = maximumDropHeight > 0f;
            sources[i] = source;
        }
        var settings = CreateBuildSettings(maximumDropHeight, agentRadius);
        agentType = settings.agentTypeID;
        filter = new NavMeshQueryFilter { agentTypeID = agentType, areaMask = NavMesh.AllAreas };
        data = new NavMeshData(agentType);
        CreateVisuals();
        initialized = true;
        Status = "Building room navigation";
        bakeOperation = NavMeshBuilder.UpdateNavMeshDataAsync(data, settings, sources, bounds);
        if (bakeOperation == null)
        {
            Status = "Could not start navigation mesh bake";
            return;
        }
        bakeOperation.completed += OnBakeCompleted;
    }

    static NavMeshBuildSettings CreateBuildSettings(float dropHeight, float radius)
    {
        var settings = NavMesh.CreateSettings();
        settings.agentRadius = radius;
        settings.agentHeight = 0.28f;
        settings.agentClimb = 0.085f;
        settings.agentSlope = 35f;
        settings.ledgeDropHeight = dropHeight;
        settings.maxJumpAcrossDistance = 0f;
        settings.overrideVoxelSize = true;
        settings.voxelSize = 0.012f;
        settings.overrideTileSize = true;
        settings.tileSize = 128;
        settings.minRegionArea = 0.01f;
        return settings;
    }

    void OnBakeCompleted(AsyncOperation operation)
    {
        if (destroying) { ReleaseResources(); return; }
        if (data == null) return;
        bakeReady = true;
        instance = NavMesh.AddNavMeshData(data);
        if (sources.Count == 0 || !instance.valid)
        {
            Status = "No walkable room surfaces — goal kept";
            return;
        }
        if (!SimulationAvailable)
        {
            Status = "Controller unavailable — goal kept";
            return;
        }
        BindAgent(controller.FootMidpoint);
        if (hasPendingPoint)
        {
            var point = pendingPoint;
            hasPendingPoint = false;
            HasDestination = false;
            AcceptDestination(point);
        }
        else Status = "Click a surface to walk there";
    }

    bool SimulationAvailable => controller != null && controller.SimulationReady;
    bool AgentReady => agent != null && agent.enabled && agent.isOnNavMesh;

    bool BindAgent(Vector3 feet)
    {
        if (!NavMesh.SamplePosition(feet, out var hit, 0.22f, filter) || Mathf.Abs(hit.position.y - feet.y) > 0.13f)
            return false;
        if (agent == null)
        {
            // The MuJoCo root is a fixed placement anchor. Only this adviser can
            // be warped when a fall lands on a disconnected navigation surface.
            var adviser = new GameObject("Navigation adviser");
            adviser.SetActive(false);
            adviser.transform.SetParent(transform, false);
            adviser.transform.position = hit.position;
            agent = adviser.AddComponent<NavMeshAgent>();
            agent.agentTypeID = agentType;
            agent.radius = agentRadius;
            agent.height = 0.28f;
            agent.baseOffset = 0f;
            agent.speed = speed;
            agent.acceleration = 0.45f;
            agent.angularSpeed = yawSpeed * Mathf.Rad2Deg;
            // Arrival is measured from physical feet below. The advisory
            // agent must not stop its predicted position ahead of the robot.
            agent.stoppingDistance = 0f;
            // Let the gait stop at the measured arrival radius. Ideal-agent
            // braking can request speeds too small to move this learned gait.
            agent.autoBraking = false;
            agent.autoRepath = true;
            agent.autoTraverseOffMeshLink = false;
            agent.updatePosition = false;
            agent.updateRotation = false;
            agent.updateUpAxis = false;
            adviser.SetActive(true);
        }
        if (!agent.Warp(hit.position)) return false;
        agent.nextPosition = hit.position;
        agent.isStopped = false;
        nextPlan = 0f;
        return true;
    }

    public bool SetDestination(Vector3 worldPoint)
    {
        if (!initialized || destroying) return false;
        if (!bakeReady)
        {
            pendingPoint = worldPoint;
            hasPendingPoint = HasDestination = true;
            return true;
        }
        return AcceptDestination(worldPoint);
    }

    bool AcceptDestination(Vector3 worldPoint)
    {
        if (!instance.valid || !NavMesh.SamplePosition(worldPoint, out var hit, 0.12f, filter) ||
            Mathf.Abs(hit.position.y - worldPoint.y) > 0.07f)
        {
            Status = HasDestination ? "Point not walkable — keeping current goal" : "Click a walkable surface";
            return false;
        }
        Destination = hit.position;
        HasDestination = true;
        previousCommand = Vector3.zero;
        bestDistance = float.PositiveInfinity;
        progressAt = Time.time;
        nextPlan = 0f;
        if (!crossingDrop)
        {
            if (SimulationAvailable) BindAgent(controller.FootMidpoint);
            RequestPath();
        }
        DrawMarker();
        return true;
    }

    void RequestPath()
    {
        nextPlan = Time.time + 0.8f;
        if (!SimulationAvailable)
        {
            Status = "Controller unavailable — goal kept";
            if (AgentReady) agent.isStopped = true;
            return;
        }
        if (!AgentReady && !BindAgent(controller.FootMidpoint))
        {
            Status = "Waiting for a walkable surface — goal kept";
            return;
        }
        agent.isStopped = false;
        agent.SetDestination(Destination);
        Status = "Planning route — goal kept";
    }

    public Vector3 Tick(float dt)
    {
        LastCommand = Vector3.zero;
        if (!initialized || !bakeReady || !instance.valid || !HasDestination) return LastCommand;
        if (!SimulationAvailable)
        {
            previousCommand = Vector3.zero;
            if (AgentReady) agent.isStopped = true;
            Status = "Controller unavailable — goal kept";
            return LastCommand;
        }
        if (hasPendingPoint)
        {
            var point = pendingPoint;
            hasPendingPoint = HasDestination = false;
            AcceptDestination(point);
            if (!HasDestination) return LastCommand;
        }
        var feet = controller.FootMidpoint;
        var verticalSpeed = (feet.y - previousFeet.y) / Mathf.Max(dt, 0.001f);
        previousFeet = feet;
        var fallen = controller.IsFallen;
        if (fallen && !previousFallen)
        {
            recovering = true;
            quietUntil = Time.time + 3f;
            progressAt = Time.time;
            previousCommand = Vector3.zero;
            if (AgentReady) agent.isStopped = true;
        }
        previousFallen = fallen;
        if (controller.RecoveryActive || !controller.ControlReady)
        {
            recovering = true;
            quietUntil = Time.time + 3f;
            progressAt = Time.time;
            if (AgentReady) agent.isStopped = true;
            previousCommand = Vector3.zero;
            Status = controller.BehaviorState + " — goal kept";
            return LastCommand;
        }
        if (recovering && (fallen || controller.Uprightness < 0.85f) && Time.time < quietUntil)
        {
            previousCommand = Vector3.zero;
            Status = "Getting up — goal kept";
            return LastCommand;
        }
        if (recovering)
        {
            recovering = false;
            progressAt = Time.time;
            bestDistance = float.PositiveInfinity;
            if (!crossingDrop) BindAgent(feet);
        }
        if (crossingDrop) return TickDrop(feet, verticalSpeed, dt);
        if (!AgentReady)
        {
            if (Time.time >= nextPlan) RequestPath();
            return LastCommand;
        }
        // Feedback never moves the duck. On ordinary surfaces Unity constrains
        // this adviser to its NavMesh; drop traversal is handled separately.
        if (controller.Uprightness >= 0.85f && Mathf.Abs(agent.nextPosition.y - feet.y) > 0.15f)
        {
            // A canceled link or an unexpected fall can leave the adviser on
            // another island. Rebind from measured support, never its old route.
            if (BindAgent(feet)) RequestPath();
        }
        agent.nextPosition = feet;
        agent.isStopped = false;
        if (!fallen && controller.Uprightness >= 0.85f &&
            Flat(Destination - feet).magnitude < ArrivalRadius && Mathf.Abs(feet.y - Destination.y) < 0.09f)
        {
            Cancel();
            Status = "Arrived";
            return LastCommand;
        }
        if (agent.isOnOffMeshLink)
        {
            var link = agent.currentOffMeshLinkData;
            if (link.valid && link.startPos.y - link.endPos.y > 0.085f &&
                link.startPos.y - link.endPos.y <= maximumDropHeight + 0.05f)
            {
                crossingDrop = true;
                leftPlatform = false;
                landingSince = -1f;
                dropStart = link.startPos;
                dropEnd = link.endPos;
                dropStarted = Time.time;
                return TickDrop(feet, verticalSpeed, dt);
            }
            Status = "Unsupported link — goal kept";
            return LastCommand;
        }
        if (agent.pathPending)
        {
            Status = "Planning route — goal kept";
            return LastCommand;
        }
        if (!agent.hasPath || agent.pathStatus != NavMeshPathStatus.PathComplete)
        {
            SetVisualColor(WaitingColor);
            if (Time.time >= nextPlan) RequestPath();
            Status = "No route yet — goal kept, retrying";
            previousCommand = Vector3.zero;
            DrawRoute(feet);
            return LastCommand;
        }
        var distance = agent.remainingDistance;
        if (distance < bestDistance - 0.025f)
        {
            bestDistance = distance;
            progressAt = Time.time;
        }
        if (Time.time - progressAt > 4f && Time.time >= nextPlan)
        {
            // Retry Unity's route from measured progress. Do not cancel a goal
            // merely because contact or recovery prevented the requested motion.
            RequestPath();
            progressAt = Time.time;
            bestDistance = float.PositiveInfinity;
        }
        Status = "Walking to target";
        SetVisualColor(RouteColor);
        LastCommand = VelocityCommand(agent.desiredVelocity, dt);
        DrawRoute(feet);
        return LastCommand;
    }

    Vector3 TickDrop(Vector3 feet, float verticalSpeed, float dt)
    {
        leftPlatform |= feet.y < dropStart.y - 0.10f;
        var onLowerSurface = NavMesh.SamplePosition(feet, out var landing, 0.14f, filter) &&
            landing.position.y < dropStart.y - 0.085f && Mathf.Abs(landing.position.y - feet.y) < 0.09f &&
            Mathf.Abs(verticalSpeed) < 0.18f &&
            Physics.Raycast(feet + Vector3.up * 0.06f, Vector3.down, 0.18f, SurfaceLayers, QueryTriggerInteraction.Ignore);
        if (leftPlatform && onLowerSurface)
        {
            if (landingSince < 0f) landingSince = Time.time;
            if (Time.time - landingSince >= 0.20f)
            {
                // Warp only the adviser after an actual landing, not the robot.
                crossingDrop = false;
                BindAgent(feet);
                RequestPath();
                progressAt = Time.time;
                bestDistance = float.PositiveInfinity;
                previousCommand = Vector3.zero;
                Status = "Landed — resuming route";
            }
        }
        else landingSince = -1f;
        if (leftPlatform)
        {
            previousCommand = Vector3.zero;
            // A fallen robot may have its feet above the floor even though its
            // trunk has landed. After the quiet recovery window, use measured
            // support below those feet to rebind the adviser and permit motion.
            // Requiring feet to be flat forever would deadlock a real fall.
            if (Time.time - dropStarted > 8f && Mathf.Abs(verticalSpeed) < 0.25f &&
                Physics.Raycast(feet + Vector3.up * 0.06f, Vector3.down, out var support, 0.45f,
                    SurfaceLayers, QueryTriggerInteraction.Ignore) && support.point.y < dropStart.y - 0.085f &&
                BindAgent(support.point))
            {
                crossingDrop = false;
                RequestPath();
                progressAt = Time.time;
                bestDistance = float.PositiveInfinity;
                Status = "Retrying from landing surface — goal kept";
                return Vector3.zero;
            }
            Status = "Landing / recovery — goal kept";
            return Vector3.zero;
        }
        if (Time.time - dropStarted > 6f)
        {
            // A failed approach can be planned again; the physical state stays.
            crossingDrop = false;
            BindAgent(feet);
            RequestPath();
            previousCommand = Vector3.zero;
            return Vector3.zero;
        }
        Status = "Walking down link — goal kept";
        var direction = Flat(dropEnd - dropStart).normalized;
        LastCommand = VelocityCommand(direction * Mathf.Min(speed, 0.30f), dt);
        return LastCommand;
    }

    Vector3 VelocityCommand(Vector3 worldVelocity, float dt)
    {
        worldVelocity = Flat(worldVelocity);
        var forward = Flat(trunk.forward).normalized;
        if (forward.sqrMagnitude < 0.001f) return Vector3.zero;
        var right = Vector3.Cross(Vector3.up, forward);
        var angle = worldVelocity.sqrMagnitude > 0.00001f
            ? Vector3.SignedAngle(forward, worldVelocity, Vector3.up) : 0f;
        // At the exactly-behind ambiguity keep the chosen turn direction until
        // a meaningful heading change, rather than alternating left and right.
        if (Mathf.Abs(angle) > 155f) angle = Mathf.Abs(angle) * turnSign;
        else if (Mathf.Abs(angle) > 5f) turnSign = Mathf.Sign(angle);
        var alignment = Mathf.Clamp01((180f - Mathf.Abs(angle)) / 90f);
        var desired = new Vector3(
            Mathf.Clamp(Vector3.Dot(worldVelocity, forward), 0f, speed) * alignment,
            Mathf.Clamp(-Vector3.Dot(worldVelocity, right), -0.12f, 0.12f) * alignment,
            Mathf.Clamp(-angle * Mathf.Deg2Rad * 0.8f, -yawSpeed, yawSpeed));
        previousCommand.x = Mathf.MoveTowards(previousCommand.x, desired.x, dt * 0.35f);
        previousCommand.y = Mathf.MoveTowards(previousCommand.y, desired.y, dt * 0.35f);
        previousCommand.z = Mathf.MoveTowards(previousCommand.z, desired.z, dt * 0.8f);
        return previousCommand;
    }

    public void Suspend()
    {
        previousCommand = LastCommand = Vector3.zero;
        if (AgentReady) agent.isStopped = true;
        if (HasDestination) Status = "Paused — goal kept";
    }

    public void Resume()
    {
        previousFeet = initialized && SimulationAvailable ? controller.FootMidpoint : Vector3.zero;
        if (HasDestination && !SimulationAvailable) Status = "Controller unavailable — goal kept";
        progressAt = Time.time;
        nextPlan = 0f;
        if (AgentReady) agent.isStopped = false;
    }

    public void Cancel()
    {
        HasDestination = hasPendingPoint = false;
        crossingDrop = false;
        previousCommand = LastCommand = Vector3.zero;
        if (AgentReady) { agent.isStopped = true; agent.ResetPath(); }
        Status = "Stopped";
        if (route != null) route.positionCount = 0;
        if (marker != null) marker.positionCount = 0;
    }

    void CreateVisuals()
    {
        var shader = Shader.Find("Universal Render Pipeline/Lit");
        if (shader == null) return;
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
        renderer.generateLightingData = true;
        renderer.shadowCastingMode = ShadowCastingMode.Off;
        renderer.receiveShadows = false;
        return renderer;
    }

    void DrawRoute(Vector3 feet)
    {
        if (route == null) return;
        if (Time.time >= nextRouteDraw)
        {
            corners = AgentReady && agent.hasPath ? agent.path.corners : System.Array.Empty<Vector3>();
            nextRouteDraw = Time.time + 0.15f;
        }
        route.positionCount = corners.Length == 0 ? 0 : corners.Length + 1;
        if (corners.Length == 0) return;
        route.SetPosition(0, feet + Vector3.up * 0.04f);
        for (var i = 0; i < corners.Length; ++i) route.SetPosition(i + 1, corners[i] + Vector3.up * 0.04f);
    }

    void DrawMarker()
    {
        if (marker == null) return;
        marker.positionCount = 33;
        for (var i = 0; i < 33; ++i)
        {
            var angle = i * Mathf.PI * 2f / 33f;
            marker.SetPosition(i, Destination + new Vector3(Mathf.Cos(angle) * 0.055f, 0.04f, Mathf.Sin(angle) * 0.055f));
        }
    }

    void SetVisualColor(Color color) { if (lineMaterial != null) lineMaterial.SetColor("_BaseColor", color); }
    static Vector3 Flat(Vector3 value) { value.y = 0f; return value; }

    void OnDestroy()
    {
        destroying = true;
        if (agent != null) agent.enabled = false;
        if (bakeOperation != null && !bakeOperation.isDone) return;
        ReleaseResources();
    }

    void ReleaseResources()
    {
        if (instance.valid) instance.Remove();
        if (data != null) { Destroy(data); data = null; }
        if (agentType != -1) { NavMesh.RemoveSettings(agentType); agentType = -1; }
        if (lineMaterial != null) { Destroy(lineMaterial); lineMaterial = null; }
        bakeOperation = null;
    }
}
