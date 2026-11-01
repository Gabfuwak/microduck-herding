using System;
using System.Collections.Generic;
using UnityEngine.AI;
using System.Runtime.InteropServices;
using System.Text;
using System.IO;
using Unity.Profiling;
using Stopwatch = System.Diagnostics.Stopwatch;
using Mujoco;
using UnityEngine;

[DisallowMultipleComponent]
[DefaultExecutionOrder(0)]
public sealed unsafe class MicroDuckPolicyController : MonoBehaviour
{
    [Serializable]
    struct NativeAsset
    {
        public string filename;
        public TextAsset data;
    }

    [Serializable]
    struct Body
    {
        public string name;
        public Transform transform;
    }

    [SerializeField] TextAsset[] controlPolicies;
    [SerializeField] TextAsset sceneXml;
    [SerializeField] NativeAsset[] nativeAssets;
    [SerializeField] Body[] bodies;
    [SerializeField] bool capturePerformance;

    const int JointCount = 14;
    const int ObjectBody = (int)MujocoLib.mjtObj.mjOBJ_BODY;
    const int ObjectJoint = (int)MujocoLib.mjtObj.mjOBJ_JOINT;
    const int ObjectActuator = (int)MujocoLib.mjtObj.mjOBJ_ACTUATOR;
    const int TransmissionJoint = (int)MujocoLib.mjtTrn.mjTRN_JOINT;
    const int FreeJoint = (int)MujocoLib.mjtJoint.mjJNT_FREE;
    const double PhysicsPeriod = 0.005;
    const int SubstepsPerPolicy = 4;
    const double CommandTimeout = 0.5;
    const double SurfaceRefreshPeriod = 0.25;

    static readonly ProfilerMarker SurfaceScanMarker = new("MicroDuck.SurfaceScan");
    static readonly ProfilerMarker SceneXmlMarker = new("MicroDuck.SceneXml");
    static readonly ProfilerMarker VfsMarker = new("MicroDuck.VfsPreparation");
    static readonly ProfilerMarker ModelCompileMarker = new("MicroDuck.ModelCompile");
    static readonly ProfilerMarker PolicyMarker = new("MicroDuck.Policy");
    static readonly ProfilerMarker SurfacePoseMarker = new("MicroDuck.SurfacePose");

    static readonly string[] JointNames =
    {
        "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
        "neck_pitch", "head_pitch", "head_yaw", "head_roll",
        "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle"
    };

    static readonly double[] DefaultPositions =
    {
        0, -0.0873, -0.4579, -0.0049, 0.453, 0.3491, 0.3491,
        0, 0, 0, 0.0873, 0.4579, 0.0049, -0.453
    };

    readonly int[] jointQpos = new int[JointCount];
    readonly int[] jointDof = new int[JointCount];
    readonly int[] actuatorIds = new int[JointCount];
    int[] bodyIds;
    Vector3 nativeGravity;
    readonly double[] baseGain = new double[JointCount];
    readonly double[] baseBiasPosition = new double[JointCount];
    MicroDuckPollenControl pollen;
    MicroDuckPollenControl.MotorOutput motorOutput;
    double appliedGainRatio = double.NaN;

    internal bool SimulationReady => initialized && data != null && pollen != null;
    public bool ControlReady => pollen != null && motorOutput.ControlReady;
    public bool RecoveryActive => pollen != null && motorOutput.RecoveryActive;
    public bool IsFallen => pollen != null && motorOutput.IsFallen;
    public float Uprightness { get; private set; } = 1f;
    public string BehaviorState => pollen == null ? "not initialized" : motorOutput.BehaviorState;
    MujocoLib.mjModel_* model;
    MujocoLib.mjData_* data;
    int rootBodyId = -1;
    int rootQposAddress = -1;
    int leftFootSiteId = -1;
    int rightFootSiteId = -1;
    int gyroAddress = -1;
    int substepsSincePolicy;
    bool initialized;
    Vector3 requestedCommand;
    double commandUpdatedAt = double.NegativeInfinity;
    MicroDuckSurfaces surfaces;
    double nextSurfaceRefresh;
    MicroDuckPerformance performance;
    long previousFrameTimestamp;

    [ContextMenu("Begin Performance Capture")]
    public void BeginPerformanceCapture()
    {
        performance = new MicroDuckPerformance();
        previousFrameTimestamp = 0;
    }

    public string GetPerformanceReport()
    {
        if (performance == null)
            throw new InvalidOperationException("Begin a performance capture before requesting its report.");
        return performance.ToJson();
    }

    [ContextMenu("Save Performance Capture")]
    public void SavePerformanceCapture()
    {
        var path = Path.Combine(Application.persistentDataPath, "microduck-performance.json");
        Directory.CreateDirectory(Application.persistentDataPath);
        File.WriteAllText(path, GetPerformanceReport());
        Debug.Log($"MicroDuck performance capture saved to {path}", this);
    }

    void LateUpdate()
    {
        if (performance == null || !initialized)
            return;
        var now = Stopwatch.GetTimestamp();
        if (previousFrameTimestamp != 0)
            performance.RecordFrame((float)((now - previousFrameTimestamp) / (double)Stopwatch.Frequency));
        previousFrameTimestamp = now;
    }

    void OnApplicationPause(bool paused)
    {
        previousFrameTimestamp = 0;
        if (paused && performance != null && capturePerformance)
            SavePerformanceCapture();
    }

    // Robot-relative m/s, m/s, rad/s. Call continuously; stale requests stop walking.
    public void SetVelocityCommand(float forward, float left, float yawRate)
    {
        requestedCommand = new Vector3(forward, left, yawRate);
        commandUpdatedAt = data != null ? data->time : 0;
    }

    public Transform Trunk { get; private set; }

    public Vector3 FootMidpoint
    {
        get
        {
            if (!initialized || data == null || leftFootSiteId < 0 || rightFootSiteId < 0)
                throw new InvalidOperationException("MicroDuck foot sites are not resolved.");
            var left = data->site_xpos + leftFootSiteId * 3;
            var right = data->site_xpos + rightFootSiteId * 3;
            var midpoint = new Vector3((float)(-(left[1] + right[1]) * 0.5),
                (float)((left[2] + right[2]) * 0.5), (float)((left[0] + right[0]) * 0.5));
            return transform.TransformPoint(midpoint);
        }
    }


    internal void GetNavigationBuildSources(List<NavMeshBuildSource> output, out Bounds bounds)
    {
        if (!initialized || surfaces == null)
            throw new InvalidOperationException("MicroDuck surfaces are not initialized.");
        surfaces.GetNavigationBuildSources(output, out bounds);
    }


    void OnEnable()
    {
        if (Application.isPlaying)
            TryInitializeSimulation();
    }

    void FixedUpdate()
    {
        if (!initialized)
            return;
        try
        {
            SimulateStep(Time.fixedDeltaTime);
        }
        catch (Exception exception)
        {
            Debug.LogException(exception, this);
            enabled = false;
        }
    }

    void TryInitializeSimulation()
    {
        try
        {
            InitializeSimulation();
        }
        catch (Exception exception)
        {
            Debug.LogException(exception, this);
            enabled = false;
        }
    }

    public void InitializeSimulation()
    {
        if (initialized)
            return;
        try
        {
            if (capturePerformance && performance == null)
                BeginPerformanceCapture();
            if (MujocoLib.mjVERSION_HEADER != 3015000 || MujocoLib.mj_version() != 3015000)
                throw new InvalidOperationException("MicroDuck requires the MuJoCo 3.15.0 runtime and bindings.");
            if (controlPolicies == null || controlPolicies.Length == 0 || sceneXml == null ||
                nativeAssets == null || bodies == null || bodies.Length == 0)
                throw new InvalidOperationException("Locomotion requires packaged Pollen policies, native scene assets and body transforms.");
            ValidateAnchor();

            pollen = new MicroDuckPollenControl(controlPolicies);

            surfaces = new MicroDuckSurfaces(transform);
            ScanSurfaces();
            model = LoadNativeModel(BuildNativeSceneXml());
            ResolveModelElements();
            data = MujocoLib.mj_makeData(model);
            if (data == null)
                throw new InvalidOperationException("MuJoCo could not allocate MicroDuck simulation data.");
            InitializeNativeState();
            surfaces.BindNativeModel(model);
            initialized = true;
            ApplyInitialState();
        }
        catch
        {
            ReleaseSimulation();
            throw;
        }
    }

    public void PlacePreparedSimulation()
    {
        if (!initialized)
            throw new InvalidOperationException("Initialize the duck before placing it.");
        ValidateAnchor();
        // Move the prepared simulation to its placement anchor without rebuilding resources.
        // Contacts, gravity, actions and clocks belong to the new initial state.
        // Reset the controller only for a new explicit placement. Falls and room
        // changes retain its state and never reset the physical robot.
        pollen.ResetPlacement();
        MujocoLib.mj_resetData(model, data);
        InitializeNativeState();
        RefreshSurfaces();
        ApplyInitialState();
    }

    void ApplyInitialState()
    {
        surfaces.ApplyNativePoses(data, true);
        MujocoLib.mj_forward(model, data);
        ApplyNativePoses();
        StepPolicy();
    }

    void ValidateAnchor()
    {
        if (Physics.gravity.sqrMagnitude < 0.000001f)
            throw new InvalidOperationException("MicroDuck native simulation requires nonzero world gravity.");
        if ((transform.lossyScale - Vector3.one).sqrMagnitude > 0.000001f)
            throw new InvalidOperationException("MicroDuck native simulation requires unit world scale.");
    }

    unsafe MujocoLib.mjModel_* LoadNativeModel(string xml)
    {
        byte* vfsMemory = stackalloc byte[sizeof(IntPtr)];
        for (var i = 0; i < sizeof(IntPtr); i++)
            vfsMemory[i] = 0;
        var vfs = vfsMemory;
        MujocoLib.mj_defaultVFS(vfs);
        try
        {
            var vfsStarted = performance != null ? Stopwatch.GetTimestamp() : 0;
            using (VfsMarker.Auto())
            {
                foreach (var asset in nativeAssets)
                {
                    if (string.IsNullOrEmpty(asset.filename) || asset.data == null)
                        throw new InvalidOperationException("Every native asset requires a filename and byte asset.");
                    var bytes = asset.data.bytes;
                    if (bytes.Length == 0)
                        throw new InvalidOperationException("Native asset is empty: " + asset.filename);
                    fixed (byte* buffer = bytes)
                        if (MujocoLib.mj_addBufferVFS(vfs, asset.filename, buffer, bytes.Length) != 0)
                            throw new InvalidOperationException("Could not add native asset to MuJoCo VFS: " + asset.filename);
                }
                var xmlBytes = Encoding.UTF8.GetBytes(xml);
                fixed (byte* buffer = xmlBytes)
                    if (MujocoLib.mj_addBufferVFS(vfs, "scene.xml", buffer, xmlBytes.Length) != 0)
                        throw new InvalidOperationException("Could not add scene XML to MuJoCo VFS.");
            }
            performance?.Record(MicroDuckPerformance.Timing.Vfs, vfsStarted);
            var error = new StringBuilder(2048);
            var compileStarted = performance != null ? Stopwatch.GetTimestamp() : 0;
            MujocoLib.mjModel_* loadedModel;
            using (ModelCompileMarker.Auto())
                loadedModel = MujocoLib.mj_loadXML("scene.xml", vfs, error, error.Capacity);
            performance?.Record(MicroDuckPerformance.Timing.ModelCompile, compileStarted);
            if (loadedModel == null)
                throw new InvalidOperationException("MuJoCo could not load MicroDuck scene: " + error);
            return loadedModel;
        }
        finally
        {
            MujocoLib.mj_deleteVFS(vfs);
        }
    }

    string BuildNativeSceneXml()
    {
        var started = performance != null ? Stopwatch.GetTimestamp() : 0;
        string xml;
        using (SceneXmlMarker.Auto())
            xml = surfaces.BuildSceneXml(sceneXml.text);
        performance?.Record(MicroDuckPerformance.Timing.SceneXml, started);
        return xml;
    }

    unsafe void ResolveModelElements()
    {
        if (model->nq < 7 || model->nv < 6 || model->nu != JointCount ||
            model->njnt != JointCount + 1)
            throw new InvalidOperationException("MicroDuck scene must have one free root, 14 joints and 14 actuators.");

        rootBodyId = -1;
        rootQposAddress = -1;

        for (var jointId = 0; jointId < (int)model->njnt; jointId++)
        {
            if (model->jnt_type[jointId] != FreeJoint)
                continue;
            if (rootBodyId >= 0)
                throw new InvalidOperationException("MicroDuck scene has more than one free root joint.");
            rootBodyId = model->jnt_bodyid[jointId];
            rootQposAddress = model->jnt_qposadr[jointId];
        }
        if (rootBodyId < 0 || model->jnt_type[0] != FreeJoint)
            throw new InvalidOperationException("MicroDuck scene requires a leading free root joint.");

        for (var i = 0; i < JointCount; i++)
        {
            var jointId = MujocoLib.mj_name2id(model, ObjectJoint, JointNames[i]);
            if (jointId < 0 || model->jnt_type[jointId] != 3)
                throw new InvalidOperationException("Missing revolute MuJoCo joint: " + JointNames[i]);
            jointQpos[i] = model->jnt_qposadr[jointId];
            jointDof[i] = model->jnt_dofadr[jointId];
            var actuatorId = -1;
            for (var candidate = 0; candidate < (int)model->nu; candidate++)
            {
                if (model->actuator_trntype[candidate] != TransmissionJoint ||
                    model->actuator_trnid[candidate * 2] != jointId)
                    continue;
                if (actuatorId >= 0)
                    throw new InvalidOperationException("Multiple actuators drive " + JointNames[i] + ".");
                actuatorId = candidate;
            }
            if (actuatorId < 0 ||
                Marshal.PtrToStringAnsi(MujocoLib.mj_id2name(model, ObjectActuator, actuatorId)) != JointNames[i])
                throw new InvalidOperationException("Missing joint-transmission actuator: " + JointNames[i]);
            actuatorIds[i] = actuatorId;
        }
        for (var i = 0; i < JointCount; i++)
            for (var j = i + 1; j < JointCount; j++)
                if (actuatorIds[i] == actuatorIds[j])
                    throw new InvalidOperationException("MuJoCo actuator transmission mapping is not one-to-one.");

        for (var i = 0; i < JointCount; i++)
        {
            baseGain[i] = model->actuator_gainprm[actuatorIds[i] * MujocoLib.mjNGAIN];
            baseBiasPosition[i] = model->actuator_biasprm[actuatorIds[i] * MujocoLib.mjNBIAS + 1];
        }
        appliedGainRatio = double.NaN;
        var gyroId = MujocoLib.mj_name2id(model, (int)MujocoLib.mjtObj.mjOBJ_SENSOR, "imu_ang_vel");
        if (gyroId < 0 || model->sensor_dim[gyroId] != 3 ||
            model->sensor_type[gyroId] != (int)MujocoLib.mjtSensor.mjSENS_GYRO)
            throw new InvalidOperationException("MicroDuck requires the source imu_ang_vel gyro sensor.");
        gyroAddress = model->sensor_adr[gyroId];

        bodyIds = new int[bodies.Length];
        Trunk = null;
        for (var i = 0; i < bodies.Length; i++)
        {
            if (string.IsNullOrEmpty(bodies[i].name) || bodies[i].transform == null)
                throw new InvalidOperationException("Every native body requires a name and visual transform.");
            var bodyId = MujocoLib.mj_name2id(model, ObjectBody, bodies[i].name);
            if (bodyId <= 0)
                throw new InvalidOperationException("Missing MuJoCo body: " + bodies[i].name);
            if (i > 0 && bodyId <= bodyIds[i - 1])
                throw new InvalidOperationException("MicroDuck visual body transforms must be ordered parent-first.");
            bodyIds[i] = bodyId;
            if (bodyId == rootBodyId)
                Trunk = bodies[i].transform;
        }
        if (Trunk == null)
            throw new InvalidOperationException("The native trunk body requires a matching visual transform.");
        var siteObject = (int)MujocoLib.mjtObj.mjOBJ_SITE;
        leftFootSiteId = MujocoLib.mj_name2id(model, siteObject, "left_foot");
        rightFootSiteId = MujocoLib.mj_name2id(model, siteObject, "right_foot");
        if (leftFootSiteId < 0 || rightFootSiteId < 0)
            throw new InvalidOperationException("MicroDuck requires native left_foot and right_foot sites.");
    }

    unsafe void InitializeNativeState()
    {
        requestedCommand = Vector3.zero;
        commandUpdatedAt = double.NegativeInfinity;
        substepsSincePolicy = 0;
        nextSurfaceRefresh = SurfaceRefreshPeriod;
        previousFrameTimestamp = 0;
        var gravityInAnchorUnity = transform.InverseTransformDirection(Physics.gravity);
        nativeGravity = new Vector3(
            gravityInAnchorUnity.z,
            -gravityInAnchorUnity.x,
            gravityInAnchorUnity.y);
        ConfigurePhysics(model);

        data->qpos[rootQposAddress] = 0;
        data->qpos[rootQposAddress + 1] = 0;
        data->qpos[rootQposAddress + 2] = 0.119182773;
        data->qpos[rootQposAddress + 3] = 1;
        data->qpos[rootQposAddress + 4] = 0;
        data->qpos[rootQposAddress + 5] = 0;
        data->qpos[rootQposAddress + 6] = 0;
        for (var i = 0; i < JointCount; i++)
        {
            data->qpos[jointQpos[i]] = DefaultPositions[i];
            data->ctrl[actuatorIds[i]] = DefaultPositions[i];
        }
    }

    void ConfigurePhysics(MujocoLib.mjModel_* nativeModel)
    {
        nativeModel->opt.timestep = PhysicsPeriod;
        nativeModel->opt.gravity[0] = nativeGravity.x;
        nativeModel->opt.gravity[1] = nativeGravity.y;
        nativeModel->opt.gravity[2] = nativeGravity.z;
    }

    MicroDuckSurfaces.Change ScanSurfaces()
    {
        var started = performance != null ? Stopwatch.GetTimestamp() : 0;
        MicroDuckSurfaces.Change change;
        using (SurfaceScanMarker.Auto())
            change = surfaces.Refresh();
        performance?.Record(MicroDuckPerformance.Timing.SurfaceScan, started);
        return change;
    }

    // Geometry recompiles do not restart the policy or discard integration history.
    // Returns whether geometry or placement changed; pose-only updates never compile.
    public bool RefreshSurfaces()
    {
        if (!initialized)
            throw new InvalidOperationException("InitializeSimulation must be called before RefreshSurfaces.");
        var change = ScanSurfaces();
        if (change == MicroDuckSurfaces.Change.None)
        {
            performance?.RecordSurfaceRefresh(false, 0);
            return false;
        }
        const MicroDuckSurfaces.Change rebuild = MicroDuckSurfaces.Change.Geometry |
            MicroDuckSurfaces.Change.Added | MicroDuckSurfaces.Change.Removed;
        if ((change & rebuild) == 0)
        {
            var started = performance != null ? Stopwatch.GetTimestamp() : 0;
            int updated;
            using (SurfacePoseMarker.Auto())
            {
                updated = surfaces.ApplyNativePoses(data, false);
                ForwardPreservingWarmstart(model, data);
            }
            performance?.Record(MicroDuckPerformance.Timing.SurfacePose, started);
            performance?.RecordSurfaceRefresh(false, updated);
            return updated != 0;
        }

        var replacementModel = LoadNativeModel(BuildNativeSceneXml());
        MujocoLib.mjData_* replacementData = null;
        try
        {
            // Surface count changes both mocap state and body-sized Cartesian forces.
            // Transfer the robot-sized packed state, then map robot forces by name.
            const int stateMask = (int)MujocoLib.mjtState.mjSTATE_INTEGRATION &
                ~((int)MujocoLib.mjtState.mjSTATE_MOCAP_POS |
                  (int)MujocoLib.mjtState.mjSTATE_MOCAP_QUAT |
                  (int)MujocoLib.mjtState.mjSTATE_XFRC_APPLIED);
            var stateSize = MujocoLib.mj_stateSize(model, stateMask);
            if (replacementModel->nq != model->nq || replacementModel->nv != model->nv ||
                replacementModel->nu != model->nu || replacementModel->na != model->na ||
                replacementModel->njnt != model->njnt || replacementModel->neq != model->neq ||
                replacementModel->nhistory != model->nhistory || replacementModel->nuserdata != model->nuserdata ||
                replacementModel->npluginstate != model->npluginstate ||
                MujocoLib.mj_stateSize(replacementModel, stateMask) != stateSize)
                throw new InvalidOperationException("Surface updates must not change the robot's integration-state layout.");
            replacementData = MujocoLib.mj_makeData(replacementModel);
            if (replacementData == null)
                throw new InvalidOperationException("MuJoCo could not allocate updated surface simulation data.");
            var state = new double[stateSize];
            fixed (double* buffer = state)
            {
                MujocoLib.mj_getState(model, data, buffer, stateMask);
                MujocoLib.mj_setState(replacementModel, replacementData, buffer, stateMask);
            }
            for (var i = 0; i < bodies.Length; i++)
            {
                var replacementBody = MujocoLib.mj_name2id(replacementModel, ObjectBody, bodies[i].name);
                if (replacementBody <= 0)
                    throw new InvalidOperationException("Missing robot body after surface update: " + bodies[i].name);
                for (var axis = 0; axis < 6; axis++)
                    replacementData->xfrc_applied[replacementBody * 6 + axis] =
                        data->xfrc_applied[bodyIds[i] * 6 + axis];
            }
            ConfigurePhysics(replacementModel);
            surfaces.BindNativeModel(replacementModel);
            surfaces.ApplyNativePoses(replacementData, true);
            ForwardPreservingWarmstart(replacementModel, replacementData);
        }
        catch
        {
            if (replacementData != null)
                MujocoLib.mj_deleteData(replacementData);
            MujocoLib.mj_deleteModel(replacementModel);
            throw;
        }

        MujocoLib.mj_deleteData(data);
        MujocoLib.mj_deleteModel(model);
        model = replacementModel;
        data = replacementData;
        ResolveModelElements();
        ApplyGainRatio(motorOutput.GainRatio);
        ApplyNativePoses();
        performance?.RecordSurfaceRefresh(true, 0);
        performance?.RecordRebuildReasons(
            (change & MicroDuckSurfaces.Change.Geometry) != 0,
            (change & MicroDuckSurfaces.Change.Added) != 0,
            (change & MicroDuckSurfaces.Change.Removed) != 0);
        return true;
    }

    static void ForwardPreservingWarmstart(MujocoLib.mjModel_* nativeModel, MujocoLib.mjData_* nativeData)
    {
        // Forward refreshes contacts/sensors without advancing time, but also
        // overwrites warmstart. Keep that integration state until the next step.
        var count = checked((int)nativeModel->nv);
        double* warmstart = stackalloc double[count];
        for (var i = 0; i < count; i++) warmstart[i] = nativeData->qacc_warmstart[i];
        MujocoLib.mj_forward(nativeModel, nativeData);
        for (var i = 0; i < count; i++) nativeData->qacc_warmstart[i] = warmstart[i];
    }

    public unsafe void SimulateStep(float deltaTime)
    {
        if (!initialized)
            throw new InvalidOperationException("InitializeSimulation must be called before SimulateStep.");
        var steps = deltaTime / (float)PhysicsPeriod;
        var roundedSteps = Mathf.RoundToInt(steps);
        if (float.IsNaN(deltaTime) || float.IsInfinity(deltaTime) || deltaTime <= 0f ||
            Mathf.Abs(steps - roundedSteps) > 0.00001f || roundedSteps < 1)
            throw new ArgumentOutOfRangeException(nameof(deltaTime), "Native step duration must be a positive integer multiple of 0.005 seconds.");
        if (data->time >= nextSurfaceRefresh)
        {
            RefreshSurfaces();
            nextSurfaceRefresh = data->time + SurfaceRefreshPeriod;
        }
        for (var step = 0; step < roundedSteps; step++)
        {
            if (substepsSincePolicy == SubstepsPerPolicy)
            {
                StepPolicy();
                substepsSincePolicy = 0;
            }
            MujocoLib.mj_step(model, data);
            substepsSincePolicy++;
        }
        ApplyNativePoses();
    }

    // Pollen owns observation construction, command filtering, policy history,
    // gains and fall transitions. This adapter only exchanges native robot data.
    unsafe void StepPolicy()
    {
        var started = performance != null ? Stopwatch.GetTimestamp() : 0;
        using var marker = PolicyMarker.Auto();
        var sensors = new MicroDuckPollenControl.SensorSnapshot();
        for (var i = 0; i < JointCount; i++)
        {
            // Upstream's physical order includes a mouth slot at index 9; the
            // imported simulation has the same 14 active joints and no mouth.
            var wire = i < 9 ? i : i + 1;
            sensors.positions[wire] = data->qpos[jointQpos[i]];
            sensors.velocities[wire] = data->qvel[jointDof[i]];
            // Same development-simulator load proxy as the daemon adapter.
            sensors.currents_ma[wire] = Math.Abs(data->actuator_force[actuatorIds[i]]) * 100.0;
        }
        var gravity = nativeGravity.normalized;
        for (var i = 0; i < 3; i++)
        {
            sensors.gyro[i] = data->sensordata[gyroAddress + i];
            sensors.gravity[i] = data->xmat[rootBodyId * 9 + i] * gravity.x +
                data->xmat[rootBodyId * 9 + 3 + i] * gravity.y +
                data->xmat[rootBodyId * 9 + 6 + i] * gravity.z;
        }
        for (var i = 0; i < 4; i++) sensors.quat[i] = data->xquat[rootBodyId * 4 + i];
        Uprightness = (float)-sensors.gravity[2];
        var requested = data->time - commandUpdatedAt <= CommandTimeout ? requestedCommand : Vector3.zero;
        var velocity = new MicroDuckPollenControl.VelocityCommand {
            forward = requested.x, left = requested.y, yaw_rate = requested.z
        };
        pollen.Step(ref sensors, ref velocity, checked((ulong)Math.Round(data->time * 1_000_000.0)), out motorOutput);
        ApplyGainRatio(motorOutput.GainRatio);
        fixed (double* targets = motorOutput.targets)
            for (var i = 0; i < JointCount; i++)
            {
                var target = targets[i < 9 ? i : i + 1];
                if (double.IsNaN(target) || double.IsInfinity(target))
                    throw new InvalidOperationException("Pollen returned a non-finite joint target.");
                data->ctrl[actuatorIds[i]] = target;
            }
        performance?.Record(MicroDuckPerformance.Timing.Policy, started);
    }

    void ApplyGainRatio(double ratio)
    {
        if (appliedGainRatio == ratio) return;
        for (var i = 0; i < JointCount; i++)
        {
            var actuator = actuatorIds[i];
            model->actuator_gainprm[actuator * MujocoLib.mjNGAIN] = baseGain[i] * ratio;
            model->actuator_biasprm[actuator * MujocoLib.mjNBIAS + 1] = baseBiasPosition[i] * ratio;
        }
        appliedGainRatio = ratio;
    }

    unsafe void ApplyNativePoses()
    {
        var anchorMatrix = transform.localToWorldMatrix;
        var anchorRotation = transform.rotation;
        for (var i = 0; i < bodies.Length; i++)
        {
            var bodyId = bodyIds[i];
            var y = data->xpos[bodyId * 3 + 1];
            var z = data->xpos[bodyId * 3 + 2];
            var qw = data->xquat[bodyId * 4];
            var qx = data->xquat[bodyId * 4 + 1];
            var qy = data->xquat[bodyId * 4 + 2];
            var qz = data->xquat[bodyId * 4 + 3];
            var x = data->xpos[bodyId * 3];
            var localPosition = new Vector3((float)-y, (float)z, (float)x);
            var localRotation = new Quaternion((float)qy, (float)-qz, (float)-qx, (float)qw);
            bodies[i].transform.SetPositionAndRotation(
                anchorMatrix.MultiplyPoint3x4(localPosition),
                anchorRotation * localRotation);
        }
    }

    public void ReleaseSimulation()
    {
        initialized = false;
        previousFrameTimestamp = 0;
        pollen?.Dispose();
        pollen = null;
        motorOutput = default;
        appliedGainRatio = double.NaN;
        surfaces?.Dispose();
        surfaces = null;
        nextSurfaceRefresh = 0;
        if (data != null)
        {
            MujocoLib.mj_deleteData(data);
            data = null;
        }
        if (model != null)
        {
            MujocoLib.mj_deleteModel(model);
            model = null;
        }
        rootBodyId = -1;
        rootQposAddress = -1;
        gyroAddress = -1;
        leftFootSiteId = -1;
        rightFootSiteId = -1;
        bodyIds = null;
        Trunk = null;
        substepsSincePolicy = 0;
    }

    void OnDisable() => ReleaseSimulation();
    void OnDestroy() => ReleaseSimulation();
}
