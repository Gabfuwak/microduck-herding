using System;
using Unity.InferenceEngine;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class MicroDuckPolicyController : MonoBehaviour
{
    [Serializable]
    struct Joint
    {
        public ArticulationBody body;
        public float defaultPosition;
    }

    [SerializeField] ModelAsset standingModel;
    [SerializeField] ModelAsset walkingModel;
    [SerializeField] ArticulationBody root;
    [SerializeField] Joint[] joints;
    [SerializeField, Min(0f)] float stiffness = 0.55f;
    [SerializeField, Min(0f)] float damping = 0.053f;
    [SerializeField, Min(0f)] float torqueLimit = 0.96f;

    const int JointCount = 14;
    const int ObservationCount = 61;
    const float ControlPeriod = 0.02f;
    Worker standingWorker;
    Worker walkingWorker;
    Tensor<float> input;
    readonly float[] lastAction = new float[JointCount];
    float elapsed;
    bool initialized;
    Vector3 command;

    public void SetVelocityCommand(float forward, float left, float yawRate) =>
        command = new Vector3(forward, left, yawRate);

    void OnEnable()
    {
        try
        {
            if (standingModel == null || walkingModel == null || root == null ||
                !root.isRoot || root.immovable || joints == null || joints.Length != JointCount)
                throw new InvalidOperationException("Locomotion requires both policies, a free root and 14 ordered joints.");
            if (Time.fixedDeltaTime > 0.00501f)
                throw new InvalidOperationException("MicroDuck locomotion requires a physics timestep of 0.005 seconds or less.");
            foreach (var joint in joints)
                if (joint.body == null || joint.body.jointType != ArticulationJointType.RevoluteJoint)
                    throw new InvalidOperationException("Locomotion requires 14 revolute joints in policy order.");

            if (!initialized)
            {
                foreach (var joint in joints)
                    joint.body.jointPosition = new ArticulationReducedSpace(joint.defaultPosition);
                initialized = true;
            }

            standingWorker = CreateWorker(standingModel);
            walkingWorker = CreateWorker(walkingModel);
            input = new Tensor<float>(new TensorShape(1, ObservationCount));
            Array.Clear(lastAction, 0, lastAction.Length);
            elapsed = 0f;
            foreach (var joint in joints)
            {
                var drive = joint.body.xDrive;
                drive.driveType = ArticulationDriveType.Force;
                drive.stiffness = stiffness;
                drive.damping = damping;
                drive.forceLimit = torqueLimit;
                drive.target = joint.defaultPosition * Mathf.Rad2Deg;
                drive.targetVelocity = 0f;
                joint.body.xDrive = drive;
            }
            StepPolicy();
            RunPolicy(walkingWorker);
        }
        catch (Exception exception)
        {
            Debug.LogException(exception, this);
            enabled = false;
        }
    }

    void FixedUpdate()
    {
        elapsed += Time.fixedDeltaTime;
        if (elapsed + 0.000001f < ControlPeriod)
            return;
        elapsed = Mathf.Max(0f, elapsed - ControlPeriod);
        try
        {
            StepPolicy();
        }
        catch (Exception exception)
        {
            Debug.LogException(exception, this);
            enabled = false;
        }
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
        worker.Schedule(input);
        var output = worker.PeekOutput() as Tensor<float>;
        if (output == null || output.shape != new TensorShape(1, JointCount))
            throw new InvalidOperationException("Locomotion policy must produce float actions [1,14].");
        output.CompleteAllPendingOperations();
        return output;
    }

    void StepPolicy()
    {
        input.CompleteAllPendingOperations();
        var gyro = root.transform.InverseTransformDirection(root.angularVelocity);
        var gravity = root.transform.InverseTransformDirection(Physics.gravity.normalized);
        // ROS->Unity reflects handedness: angular vectors need the opposite sign to polar vectors.
        input[0] = -gyro.z;
        input[1] = gyro.x;
        input[2] = -gyro.y;
        input[3] = gravity.z;
        input[4] = -gravity.x;
        input[5] = gravity.y;
        for (var i = 0; i < JointCount; i++)
        {
            input[6 + i] = joints[i].body.jointPosition[0] - joints[i].defaultPosition;
            input[20 + i] = joints[i].body.jointVelocity[0];
            input[34 + i] = lastAction[i];
        }
        input[48] = command.x;
        input[49] = command.y;
        input[50] = command.z;
        // Head and body-pose command slots remain zero.
        var output = RunPolicy(command.sqrMagnitude > 0f ? walkingWorker : standingWorker);
        var actions = output.AsReadOnlyNativeArray();
        for (var i = 0; i < JointCount; i++)
            if (float.IsNaN(actions[i]) || float.IsInfinity(actions[i]))
                throw new InvalidOperationException("Locomotion policy produced a non-finite joint action.");
        for (var i = 0; i < JointCount; i++)
        {
            lastAction[i] = actions[i];
            var drive = joints[i].body.xDrive;
            drive.target = (joints[i].defaultPosition + actions[i]) * Mathf.Rad2Deg;
            joints[i].body.xDrive = drive;
        }
    }

    void OnDisable()
    {
        standingWorker?.Dispose();
        standingWorker = null;
        walkingWorker?.Dispose();
        walkingWorker = null;
        input?.Dispose();
        input = null;
        if (joints == null)
            return;
        foreach (var joint in joints)
        {
            if (joint.body == null)
                continue;
            var drive = joint.body.xDrive;
            drive.stiffness = 0f;
            drive.damping = 0f;
            drive.forceLimit = 0f;
            joint.body.xDrive = drive;
        }
    }
}
