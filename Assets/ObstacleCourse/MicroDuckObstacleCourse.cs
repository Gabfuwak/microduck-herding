using UnityEngine;
using UnityEngine.InputSystem;

// Desktop controls for the separate obstacle scene; the shared robot prefab
// keeps its existing XR joystick controller.
[DefaultExecutionOrder(-10)]
public sealed class MicroDuckObstacleCourse : MonoBehaviour
{
    public MicroDuckPolicyController duck;
    public Transform trunk;
    public Camera view;
    float yaw = -35f;
    float pitch = 35f;
    float distance = 2.6f;
    GUIStyle titleStyle, hintStyle;
    public MicroDuckNavigator Navigator { get; private set; }
    Vector3 manualCommand;
    float previousTimeScale;

    void Start()
    {
        previousTimeScale = Time.timeScale;
        Time.timeScale = duck.UsesRobotd && !duck.RobotdLockstep ? 1f : 4f;
        // OnEnable may precede scene loading. Import the course before the first
        // physics step while preserving the already-prepared robot state.
        if (duck.enabled) duck.RefreshSurfaces();
        Navigator = gameObject.AddComponent<MicroDuckNavigator>();
        Navigator.Initialize(trunk, duck);
    }

    void Update()
    {
        var keyboard = Keyboard.current;
        bool focused = Application.isFocused && keyboard != null;
        float forward = 0, left = 0, turn = 0;
        if (focused)
        {
            if (!keyboard.spaceKey.isPressed)
            {
                forward = ((keyboard.wKey.isPressed || keyboard.upArrowKey.isPressed) ? 0.4f : 0f) -
                    ((keyboard.sKey.isPressed || keyboard.downArrowKey.isPressed) ? 0.25f : 0f);
                turn = ((keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed) ? 1f : 0f) -
                    ((keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed) ? 1f : 0f);
                left = (keyboard.qKey.isPressed ? 0.3f : 0f) - (keyboard.eKey.isPressed ? 0.3f : 0f);
            }
        }
        manualCommand = new Vector3(forward, left, turn);
        if (manualCommand.sqrMagnitude > 0 || (focused && keyboard.spaceKey.isPressed))
            Navigator.Cancel(manualCommand.sqrMagnitude > 0 ? "Manual control" : "Stopped");
        var mouse = Mouse.current;
        if (Application.isFocused && mouse != null)
        {
            if (mouse.leftButton.wasPressedThisFrame &&
                !(mouse.position.ReadValue().x < 510 && mouse.position.ReadValue().y > Screen.height - 170))
            {
                var ray = view.ScreenPointToRay(mouse.position.ReadValue());
                if (Physics.Raycast(ray, out var hit, 60f, 1 << 7, QueryTriggerInteraction.Ignore))
                    Navigator.SetDestination(hit.point);
            }
            if (mouse.rightButton.isPressed)
            {
                var delta = mouse.delta.ReadValue();
                yaw += delta.x * 0.2f;
                pitch = Mathf.Clamp(pitch - delta.y * 0.2f, 10f, 80f);
            }
            float scroll = mouse.scroll.ReadValue().y;
            if (Mathf.Abs(scroll) > 0)
                distance = Mathf.Clamp(distance - Mathf.Sign(scroll) * 0.15f, 0.55f, 6f);
        }
    }

    void FixedUpdate()
    {
        if (!Application.isFocused) { duck.SetVelocityCommand(0, 0, 0); return; }
        Vector3 command = Navigator != null && Navigator.HasDestination
            ? Navigator.Tick(Time.fixedDeltaTime) : manualCommand;
        duck.SetVelocityCommand(command.x, command.y, command.z);
    }

    void LateUpdate()
    {
        if (view == null || trunk == null) return;
        Vector3 target = trunk.position + new Vector3(0, 0.04f, 0.3f);
        view.transform.position = target + Quaternion.Euler(pitch, yaw, 0) * (Vector3.back * distance);
        view.transform.LookAt(target);
    }

    void OnApplicationFocus(bool focused)
    {
        if (focused) Navigator?.Resume();
        if (!focused && duck != null)
        {
            Navigator?.Suspend();
            manualCommand = Vector3.zero;
            duck.SetVelocityCommand(0, 0, 0);
        }
    }

    void OnDisable()
    {
        Time.timeScale = previousTimeScale;
        Navigator?.Cancel();
        if (duck != null) duck.SetVelocityCommand(0, 0, 0);
    }

    void OnGUI()
    {
        titleStyle ??= new GUIStyle(GUI.skin.label) { fontSize = 18, fontStyle = FontStyle.Bold };
        hintStyle ??= new GUIStyle(GUI.skin.label) { fontSize = 13 };
        GUI.Box(new Rect(12, 12, 500, 158), GUIContent.none);
        GUI.Label(new Rect(24, 20, 470, 26), $"MicroDuck / Click to walk · {Time.timeScale:0.#}× simulation", titleStyle);
        GUI.Label(new Rect(24, 49, 470, 22), "Left-click target · Space cancel · WASD/QE override", hintStyle);
        GUI.Label(new Rect(24, 73, 470, 22), "Physical recovery · Right-drag orbit · Scroll zoom", hintStyle);
        GUI.Label(new Rect(24, 127, 470, 24), $"{(duck.UsesRobotd ? "robotd" : "Policy")}: {duck.ControllerStatus}", hintStyle);
        GUI.Label(new Rect(24, 100, 470, 24), Navigator != null ? Navigator.Status : "Preparing navigation", hintStyle);
    }
}
