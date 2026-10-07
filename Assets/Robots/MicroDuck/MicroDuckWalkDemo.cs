using UnityEngine;

[RequireComponent(typeof(MicroDuckPolicyController))]
[DisallowMultipleComponent]
[DefaultExecutionOrder(-100)]
public sealed class MicroDuckWalkDemo : MonoBehaviour
{
    const float ForwardSpeed = 0.3f;
    MicroDuckPolicyController controller;
    double elapsed;
    bool walking;

    void Awake() => controller = GetComponent<MicroDuckPolicyController>();

    void OnEnable()
    {
        elapsed = 0d;
        walking = false;
        controller.SetVelocityCommand(0f, 0f, 0f);
    }

    void FixedUpdate()
    {
        if (walking)
            return;
        elapsed += Time.fixedDeltaTime;
        if (elapsed + 0.000001d < 5d)
            return;
        walking = true;
        controller.SetVelocityCommand(ForwardSpeed, 0f, 0f);
    }

    void OnDisable()
    {
        if (controller != null)
            controller.SetVelocityCommand(0f, 0f, 0f);
    }
}
