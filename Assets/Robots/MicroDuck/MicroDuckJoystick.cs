using UnityEngine;
using UnityEngine.InputSystem.Controls;
using UnityEngine.InputSystem.XR;

[RequireComponent(typeof(MicroDuckPolicyController))]
[DisallowMultipleComponent]
[DefaultExecutionOrder(-100)]
public sealed class MicroDuckJoystick : MonoBehaviour
{
    MicroDuckPolicyController controller;
    Vector2Control leftStick;
    Vector2Control rightStick;
    bool focused = true;
    bool paused;
    bool requireNeutral = true;

    void Awake() => controller = GetComponent<MicroDuckPolicyController>();

    void OnEnable() => Stop();

    void Update()
    {
        var left = XRController.leftHand;
        var right = XRController.rightHand;
        if (leftStick?.device != left)
            leftStick = left?.TryGetChildControl<Vector2Control>("primary2DAxis");
        if (rightStick?.device != right)
            rightStick = right?.TryGetChildControl<Vector2Control>("primary2DAxis");

        if (!focused || paused || leftStick == null || rightStick == null ||
            !left.enabled || !right.enabled || !left.isTracked.isPressed || !right.isTracked.isPressed)
        {
            Stop();
            return;
        }

        // XRI already binds the OpenXR sticks. Read raw values so its radial
        // dead-zone processor does not reshape the official per-axis dead zone.
        var move = leftStick.ReadUnprocessedValue();
        var turn = rightStick.ReadUnprocessedValue();
        var forward = Axis(move.y) * 0.4f;
        var sideways = -Axis(move.x) * 0.3f;
        var yaw = -Axis(turn.x);
        if (requireNeutral)
        {
            if (forward != 0f || sideways != 0f || yaw != 0f)
                return;
            requireNeutral = false;
        }
        controller.SetVelocityCommand(forward, sideways, yaw);
    }

    static float Axis(float value) =>
        float.IsNaN(value) || float.IsInfinity(value) || Mathf.Abs(value) < 0.1f
            ? 0f
            : Mathf.Clamp(value, -1f, 1f);

    void Stop()
    {
        requireNeutral = true;
        if (controller != null)
            controller.SetVelocityCommand(0f, 0f, 0f);
    }

    void OnApplicationFocus(bool hasFocus)
    {
        focused = hasFocus;
        if (!hasFocus)
            Stop();
    }

    void OnApplicationPause(bool isPaused)
    {
        paused = isPaused;
        if (isPaused)
            Stop();
    }

    void OnDisable() => Stop();
}
