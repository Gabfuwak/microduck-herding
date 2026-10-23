using System;
using System.Runtime.InteropServices;
using System.Text;
using System.IO;
using Unity.Profiling;
using Stopwatch = System.Diagnostics.Stopwatch;
using Mujoco;
using Unity.InferenceEngine;
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

    [SerializeField] ModelAsset standingModel;
    [SerializeField] ModelAsset walkingModel;
    [SerializeField] TextAsset sceneXml;
    [SerializeField] NativeAsset[] nativeAssets;
    [SerializeField] Body[] bodies;
    [SerializeField] bool capturePerformance;

    const int JointCount = 14;
    const int ObservationCount = 61;
    const int ObjectBody = (int)MujocoLib.mjtObj.mjOBJ_BODY;
    const int ObjectJoint = (int)MujocoLib.mjtObj.mjOBJ_JOINT;
    const int ObjectActuator = (int)MujocoLib.mjtObj.mjOBJ_ACTUATOR;
    const int TransmissionJoint = (int)MujocoLib.mjtTrn.mjTRN_JOINT;
    const int FreeJoint = (int)MujocoLib.mjtJoint.mjJNT_FREE;
    const double PhysicsPeriod = 0.005;
    const int SubstepsPerPolicy = 4;

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
    readonly float[] lastAction = new float[JointCount];
    Worker standingWorker;
    Worker walkingWorker;
    Tensor<float> input;
    MujocoLib.mjModel_* model;
    MujocoLib.mjData_* data;
    int rootBodyId = -1;
    int rootQposAddress = -1;
    int gyroAddress = -1;
    int substepsSincePolicy;
    bool initialized;
    Vector3 command;
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

    public void SetVelocityCommand(float forward, float left, float yawRate) =>
        command = new Vector3(forward, left, yawRate);

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
            if (standingModel == null || walkingModel == null || sceneXml == null ||
                nativeAssets == null || bodies == null || bodies.Length == 0)
                throw new InvalidOperationException("Locomotion requires both policies, native scene assets and body transforms.");
            if (Physics.gravity.sqrMagnitude < 0.000001f)
                throw new InvalidOperationException("MicroDuck native simulation requires nonzero world gravity.");
            if ((transform.lossyScale - Vector3.one).sqrMagnitude > 0.000001f)
                throw new InvalidOperationException("MicroDuck native simulation requires unit world scale.");

            standingWorker = CreateWorker(standingModel);
            walkingWorker = CreateWorker(walkingModel);
            input = new Tensor<float>(new TensorShape(1, ObservationCount));

            surfaces = new MicroDuckSurfaces(transform);
            var scanStarted = performance != null ? Stopwatch.GetTimestamp() : 0;
            using (SurfaceScanMarker.Auto())
                surfaces.Refresh();
            performance?.Record(MicroDuckPerformance.Timing.SurfaceScan, scanStarted);
            model = LoadNativeModel(BuildNativeSceneXml());
            ResolveModelElements();
            InitializeNativeState();
            surfaces.BindNativeModel(model);
            surfaces.ApplyNativePoses(data, true);
            MujocoLib.mj_forward(model, data);
            ApplyNativePoses();

            Array.Clear(lastAction, 0, lastAction.Length);
            substepsSincePolicy = 0;
            nextSurfaceRefresh = 0.25;
            initialized = true;
            StepPolicy();
            RunPolicy(walkingWorker);
        }
        catch
        {
            ReleaseSimulation();
            throw;
        }
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

        var gyroId = MujocoLib.mj_name2id(model, (int)MujocoLib.mjtObj.mjOBJ_SENSOR, "imu_ang_vel");
        if (gyroId < 0 || model->sensor_dim[gyroId] != 3 ||
            model->sensor_type[gyroId] != (int)MujocoLib.mjtSensor.mjSENS_GYRO)
            throw new InvalidOperationException("MicroDuck requires the source imu_ang_vel gyro sensor.");
        gyroAddress = model->sensor_adr[gyroId];

        bodyIds = new int[bodies.Length];
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
        }
    }

    unsafe void InitializeNativeState()
    {
        data = MujocoLib.mj_makeData(model);
        if (data == null)
            throw new InvalidOperationException("MuJoCo could not allocate MicroDuck simulation data.");
        model->opt.timestep = PhysicsPeriod;
        var gravityInAnchorUnity = transform.InverseTransformDirection(Physics.gravity);
        nativeGravity = new Vector3(
            gravityInAnchorUnity.z,
            -gravityInAnchorUnity.x,
            gravityInAnchorUnity.y);
        model->opt.gravity[0] = nativeGravity.x;
        model->opt.gravity[1] = nativeGravity.y;
        model->opt.gravity[2] = nativeGravity.z;

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

    // Geometry recompiles do not restart the policy or discard integration history.
    // Returns whether geometry or placement changed; pose-only updates never compile.
    public bool RefreshSurfaces()
    {
        if (!initialized)
            throw new InvalidOperationException("InitializeSimulation must be called before RefreshSurfaces.");
        var scanStarted = performance != null ? Stopwatch.GetTimestamp() : 0;
        MicroDuckSurfaces.Change change;
        using (SurfaceScanMarker.Auto())
            change = surfaces.Refresh();
        performance?.Record(MicroDuckPerformance.Timing.SurfaceScan, scanStarted);
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
            replacementModel->opt.timestep = PhysicsPeriod;
            replacementModel->opt.gravity[0] = nativeGravity.x;
            replacementModel->opt.gravity[1] = nativeGravity.y;
            replacementModel->opt.gravity[2] = nativeGravity.z;
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
            nextSurfaceRefresh = data->time + 0.25;
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

    static Worker CreateWorker(ModelAsset asset)
    {
        var network = ModelLoader.Load(asset);
        if (network.inputs.Count != 1 || network.outputs.Count != 1 ||
            network.inputs[0].dataType != DataType.Float ||
            network.inputs[0].shape != new DynamicTensorShape(1, ObservationCount))
            throw new InvalidOperationException("Locomotion requires float observations [1,61] and one action output.");
        return new Worker(network, BackendType.CPU);
    }

    Tensor<float> RunPolicy(Worker worker)
    {
        var started = performance != null ? Stopwatch.GetTimestamp() : 0;
        using var marker = PolicyMarker.Auto();
        worker.Schedule(input);
        var output = worker.PeekOutput() as Tensor<float>;
        if (output == null || output.shape != new TensorShape(1, JointCount))
            throw new InvalidOperationException("Locomotion policy must produce float actions [1,14].");
        output.CompleteAllPendingOperations();
        performance?.Record(MicroDuckPerformance.Timing.Policy, started);
        return output;
    }

    unsafe void StepPolicy()
    {
        input.CompleteAllPendingOperations();
        // mjOBJ_BODY velocity is in the inertial frame, not the policy's IMU frame.
        var gyroX = (float)data->sensordata[gyroAddress];
        var gyroY = (float)data->sensordata[gyroAddress + 1];
        var gyroZ = (float)data->sensordata[gyroAddress + 2];
        var gravity = nativeGravity.normalized;
        var gravityX = gravity.x;
        var gravityY = gravity.y;
        var gravityZ = gravity.z;
        var r00 = data->xmat[rootBodyId * 9];
        var r10 = data->xmat[rootBodyId * 9 + 3];
        var r20 = data->xmat[rootBodyId * 9 + 6];
        var r01 = data->xmat[rootBodyId * 9 + 1];
        var r11 = data->xmat[rootBodyId * 9 + 4];
        var r21 = data->xmat[rootBodyId * 9 + 7];
        var r02 = data->xmat[rootBodyId * 9 + 2];
        var r12 = data->xmat[rootBodyId * 9 + 5];
        var r22 = data->xmat[rootBodyId * 9 + 8];
        input[0] = gyroX;
        input[1] = gyroY;
        input[2] = gyroZ;
        input[3] = (float)(r00 * gravityX + r10 * gravityY + r20 * gravityZ);
        input[4] = (float)(r01 * gravityX + r11 * gravityY + r21 * gravityZ);
        input[5] = (float)(r02 * gravityX + r12 * gravityY + r22 * gravityZ);
        for (var i = 0; i < JointCount; i++)
        {
            input[6 + i] = (float)(data->qpos[jointQpos[i]] - DefaultPositions[i]);
            input[20 + i] = (float)data->qvel[jointDof[i]];
            input[34 + i] = lastAction[i];
        }
        input[48] = command.x;
        input[49] = command.y;
        input[50] = command.z;
        var output = RunPolicy(command.sqrMagnitude > 0f ? walkingWorker : standingWorker);
        var actions = output.AsReadOnlyNativeArray();
        for (var i = 0; i < JointCount; i++)
            if (float.IsNaN(actions[i]) || float.IsInfinity(actions[i]))
                throw new InvalidOperationException("Locomotion policy produced a non-finite joint action.");
        for (var i = 0; i < JointCount; i++)
        {
            lastAction[i] = actions[i];
            data->ctrl[actuatorIds[i]] = DefaultPositions[i] + actions[i];
        }
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
        standingWorker?.Dispose();
        standingWorker = null;
        walkingWorker?.Dispose();
        walkingWorker = null;
        input?.Dispose();
        input = null;
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
        bodyIds = null;
        substepsSincePolicy = 0;
    }

    void OnDisable() => ReleaseSimulation();
    void OnDestroy() => ReleaseSimulation();
}
