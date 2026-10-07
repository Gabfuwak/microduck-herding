using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Xml;
using Mujoco;
using Unity.InferenceEngine;
using UnityEngine;
using UnityEngine.InputSystem;

// Uses the official MuJoCo Unity plugin's native bindings. Load the source MJCF
// directly so importing through URDF or rebuilding MJCF cannot change dynamics.
[DisallowMultipleComponent]
public sealed unsafe class MicroDuckMujocoController : MonoBehaviour
{
    public ModelAsset walkingModel;
    public ModelAsset recoveryModel;
    public Transform[] links;
    public bool demoWalking = true;
    public bool keyboardControl;
    public float forwardSpeed = 0.3f;
    public float turnSpeed = 0.4f;

    static readonly string[] JointNames = {
        "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
        "neck_pitch", "head_pitch", "head_yaw", "head_roll",
        "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle" };
    static readonly float[] DefaultPose = {
        0f, -0.0872664626f, -0.457924f, -0.004940f, 0.452984f,
        0.3490658504f, 0.3490658504f, 0f, 0f,
        0f, 0.0872664626f, 0.457924f, 0.004940f, -0.452984f };
    const double PhysicsPeriod = 0.005;
    readonly int[] qposAddresses = new int[14];
    readonly int[] dofAddresses = new int[14];
    readonly int[] actuatorIds = new int[14];
    readonly float[] lastAction = new float[14];
    int[] bodyIds;
    int gyroAddress, trunkId;
    MujocoLib.mjModel_* model;
    MujocoLib.mjData_* data;
    Worker walkingWorker, recoveryWorker;
    Tensor<float> observation;
    Vector3 command;
    double accumulator, nextReport;
    int substep;
    bool manualCommand, resetRequested;
    bool recovering;
    public double SimulationTime => data == null ? 0 : data->time;
    public bool IsUpright { get; private set; }
    public int RecoveryCount { get; private set; }
    public float Height { get; private set; }
    public Vector3 VelocityCommand => command;

    public void SetVelocityCommand(float forward, float left, float yawRate)
    {
        manualCommand = true;
        command = new Vector3(forward, left, yawRate);
    }

    public bool IsReady => data != null;
    public string InitializationError { get; private set; }

    void OnEnable()
    {
        if (!Application.isPlaying) return;
        InitializationError = null;
        StartCoroutine(MicroDuckMujocoAssets.Prepare(Initialize, FailInitialization));
    }

    void FailInitialization(Exception error)
    {
        InitializationError = error.Message;
        Debug.LogException(error, this);
        enabled = false;
    }

    void Initialize(string directory)
    {
        if (!isActiveAndEnabled) return;
        try
        {
            if (walkingModel == null || recoveryModel == null || links == null || links.Length != 15)
                throw new InvalidOperationException("MuJoCo MicroDuck requires two policies and 15 visual links.");
            string version = Marshal.PtrToStringAnsi(MujocoLib.mj_versionString());
            if (version != "3.15.0")
                throw new InvalidOperationException($"MicroDuck requires MuJoCo 3.15.0 bindings and binary; loaded {version}.");
            var xml = new XmlDocument();
            xml.Load(Path.Combine(directory, "robot.xml"));
            xml.SelectSingleNode("/mujoco/compiler").Attributes["meshdir"].Value = Path.Combine(directory, "assets");
            model = MjEngineTool.LoadModelFromString(xml.OuterXml);
            if (model->nq != 21 || model->nu != 14)
                throw new InvalidOperationException("Unexpected MicroDuck MJCF joint/actuator counts.");
            data = MujocoLib.mj_makeData(model);
            if (data == null) throw new InvalidOperationException("MuJoCo could not allocate simulation state.");
            bodyIds = new int[links.Length];
            for (int i = 0; i < links.Length; i++)
                bodyIds[i] = FindId(MujocoLib.mjtObj.mjOBJ_BODY, links[i].name);
            for (int i = 0; i < 14; i++)
            {
                int id = FindId(MujocoLib.mjtObj.mjOBJ_JOINT, JointNames[i]);
                qposAddresses[i] = model->jnt_qposadr[id];
                dofAddresses[i] = model->jnt_dofadr[id];
                actuatorIds[i] = FindId(MujocoLib.mjtObj.mjOBJ_ACTUATOR, JointNames[i]);
            }
            trunkId = FindId(MujocoLib.mjtObj.mjOBJ_BODY, "trunk_base");
            gyroAddress = model->sensor_adr[FindId(MujocoLib.mjtObj.mjOBJ_SENSOR, "imu_ang_vel")];
            walkingWorker = CreateWorker(walkingModel);
            recoveryWorker = CreateWorker(recoveryModel);
            observation = new Tensor<float>(new TensorShape(1, 61));
            ResetSimulation();
            Debug.Log("MicroDuck: native MuJoCo 3.15.0, physics 200 Hz, ONNX policy 50 Hz.", this);
        }
        catch (Exception e)
        {
            FailInitialization(e);
        }
    }

    int FindId(MujocoLib.mjtObj type, string name)
    {
        int id = MujocoLib.mj_name2id(model, (int)type, name);
        if (id < 0) throw new InvalidOperationException($"Missing MuJoCo {type}: {name}");
        return id;
    }

    static Worker CreateWorker(ModelAsset asset)
    {
        var network = ModelLoader.Load(asset);
        if (network.inputs.Count != 1 || network.outputs.Count != 1 ||
            network.inputs[0].shape != new DynamicTensorShape(1, 61))
            throw new InvalidOperationException("MicroDuck requires a [1,61] ONNX policy.");
        return new Worker(network, BackendType.CPU);
    }

    void Update()
    {
        if (keyboardControl && Keyboard.current != null && Keyboard.current.rKey.wasPressedThisFrame)
            resetRequested = true;
    }

    void FixedUpdate()
    {
        if (data == null) return;
        try
        {
            if (resetRequested) { ResetSimulation(); resetRequested = false; }
            accumulator += Time.fixedDeltaTime;
            // Policy targets are held across four physics steps, independent of
            // Unity's render rate and its global Fixed Timestep.
            while (accumulator + 1e-9 >= PhysicsPeriod)
            {
                if (substep % 4 == 0) StepPolicy();
                MujocoLib.mj_step(model, data);
                accumulator -= PhysicsPeriod;
                substep++;
            }
            SyncVisuals();
            if (data->time >= nextReport)
            {
                Debug.Log($"MicroDuck MuJoCo: t={data->time:F1}s, height={Height:F3}m, upright={IsUpright}, recovery={recovering}, falls={RecoveryCount}, position=({data->qpos[0]:F2},{data->qpos[1]:F2}), cmd={command}, action0={lastAction[0]:F3}, action2={lastAction[2]:F3}.", this);
                #if UNITY_EDITOR
                if (nextReport == 10)
                {
                    observation.CompleteAllPendingOperations();
                    var snapshot = new PolicySnapshot { observations = observation.DownloadToArray(), actions = (float[])lastAction.Clone() };
                    File.WriteAllText(Path.Combine(Application.dataPath, "../Library/MicroDuckPolicySnapshot.json"), JsonUtility.ToJson(snapshot));
                }
#endif
                nextReport += 10;
            }
        }
        catch (Exception e) { FailInitialization(e); }
    }

    public void ResetSimulation()
    {
        if (data == null) return;
        MujocoLib.mj_resetData(model, data);
        data->qpos[2] = 0.12;
        for (int i = 0; i < 14; i++)
        {
            data->qpos[qposAddresses[i]] = DefaultPose[i];
            data->ctrl[actuatorIds[i]] = DefaultPose[i];
        }
        Array.Clear(lastAction, 0, 14);
        accumulator = 0; substep = 0; nextReport = 10; recovering = false; RecoveryCount = 0;
        MujocoLib.mj_forward(model, data);
        SyncVisuals();
    }

    void StepPolicy()
    {
        observation.CompleteAllPendingOperations();
        // Sensor values and joint coordinates remain in native MJCF coordinates.
        // Inverse rotation of world gravity by the trunk quaternion (wxyz).
        double* q = data->xquat + 4 * trunkId;
        float w = (float)q[0], x = (float)q[1], y = (float)q[2], z = (float)q[3];
        Vector3 gravity = Quaternion.Inverse(new Quaternion(x, y, z, w)) * Vector3.back;
        for (int i = 0; i < 3; i++) observation[i] = (float)data->sensordata[gyroAddress + i];
        observation[3] = gravity.x; observation[4] = gravity.y; observation[5] = gravity.z;
        Height = (float)data->qpos[2];
        IsUpright = gravity.z < -0.85f && Height > 0.05f;
        if (!recovering && (gravity.z > -0.5f || Height < 0.02f))
        {
            recovering = true;
            RecoveryCount++;
            Debug.Log($"MicroDuck fall recovery at {data->time:F2}s.", this);
        }
        if (recovering && IsUpright) recovering = false;
        for (int i = 0; i < 14; i++)
        {
            observation[6 + i] = (float)data->qpos[qposAddresses[i]] - DefaultPose[i];
            observation[20 + i] = (float)data->qvel[dofAddresses[i]];
            observation[34 + i] = lastAction[i];
        }
        if (!manualCommand)
            command = demoWalking && data->time >= 5 ? new Vector3(forwardSpeed, 0, turnSpeed) : Vector3.zero;
        if (keyboardControl && Keyboard.current != null)
        {
            var k = Keyboard.current;
            bool arrows = k.upArrowKey.isPressed || k.downArrowKey.isPressed || k.leftArrowKey.isPressed || k.rightArrowKey.isPressed;
            if (arrows || k.spaceKey.isPressed)
                command = new Vector3((k.upArrowKey.isPressed ? 0.3f : 0) - (k.downArrowKey.isPressed ? 0.2f : 0), 0,
                    (k.leftArrowKey.isPressed ? 0.8f : 0) - (k.rightArrowKey.isPressed ? 0.8f : 0));
        }
        for (int i = 48; i < 61; i++) observation[i] = 0;
        if (!recovering)
        {
            observation[48] = command.x; observation[49] = command.y; observation[50] = command.z;
        }
        var worker = recovering ? recoveryWorker : walkingWorker;
        worker.Schedule(observation);
        var output = worker.PeekOutput() as Tensor<float>;
        if (output == null || output.shape != new TensorShape(1, 14))
            throw new InvalidOperationException("MicroDuck policy must output [1,14] actions.");
        output.CompleteAllPendingOperations();
        var actions = output.AsReadOnlyNativeArray();
        for (int i = 0; i < 14; i++)
        {
            if (float.IsNaN(actions[i]) || float.IsInfinity(actions[i]))
                throw new InvalidOperationException("Non-finite MicroDuck policy action.");
            lastAction[i] = actions[i];
            data->ctrl[actuatorIds[i]] = DefaultPose[i] + actions[i];
        }
    }

    void SyncVisuals()
    {
        // Existing URDF meshes use ROS->Unity (-y,z,x), whereas MuJoCo's
        // generic Unity importer uses (x,z,y). Match the existing mesh frames.
        for (int i = 0; i < links.Length; i++)
        {
            double* p = data->xpos + bodyIds[i] * 3;
            double* q = data->xquat + bodyIds[i] * 4;
            Vector3 position = new Vector3(-(float)p[1], (float)p[2], (float)p[0]);
            Quaternion rotation = new Quaternion((float)q[2], -(float)q[3], -(float)q[1], (float)q[0]);
            links[i].SetPositionAndRotation(transform.TransformPoint(position), transform.rotation * rotation);
        }
    }

    [Serializable]
    public class PolicySnapshot { public float[] observations; public float[] actions; }

    void OnDisable()
    {
        StopAllCoroutines();
        walkingWorker?.Dispose(); walkingWorker = null;
        recoveryWorker?.Dispose(); recoveryWorker = null;
        observation?.Dispose(); observation = null;
        if (data != null) { MujocoLib.mj_deleteData(data); data = null; }
        if (model != null) { MujocoLib.mj_deleteModel(model); model = null; }
    }
}
