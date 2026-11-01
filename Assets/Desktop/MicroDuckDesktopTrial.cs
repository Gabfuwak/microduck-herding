using UnityEngine;
using UnityEngine.InputSystem;

// A desktop-only view of the same spawner, navigation and physical controller
// used by MainScene. This component is confined to the navigation trial scene.
[RequireComponent(typeof(Camera))]
public sealed class MicroDuckDesktopTrial : MonoBehaviour
{
    [SerializeField] MicroDuckSpawner spawner;
    [SerializeField] Vector3 startingPoint = new(0f, 0.45f, -1f);
    [SerializeField] Vector3 orbitCenter = new(0f, 0.35f, 0f);
    Camera view;
    MicroDuckNavigator navigator;
    MicroDuckPolicyController controller;
    float distance = 4.2f;
    float yaw = 145f;
    float pitch = 30f;
    float previousTimeScale;
    float playingSpeed = 4f;

    void OnEnable()
    {
#if UNITY_EDITOR || UNITY_STANDALONE_LINUX
        previousTimeScale = Time.timeScale;
        Time.timeScale = playingSpeed;
#else
        enabled = false;
#endif
    }

    void Start()
    {
        view = GetComponent<Camera>();
        MoveCamera();
        spawner.cameraToFace = view;
        spawner.TryPlaceOrNavigate(startingPoint, Vector3.up);
    }

    void Update()
    {
        if (navigator == null) navigator = FindFirstObjectByType<MicroDuckNavigator>();
        if (controller == null) controller = FindFirstObjectByType<MicroDuckPolicyController>();
        var mouse = Mouse.current;
        if (mouse != null)
        {
            if (mouse.rightButton.isPressed)
            {
                var delta = mouse.delta.ReadValue();
                yaw += delta.x * 0.18f;
                pitch = Mathf.Clamp(pitch - delta.y * 0.18f, 8f, 80f);
            }
            distance = Mathf.Clamp(distance - mouse.scroll.ReadValue().y * 0.002f, 0.5f, 10f);
            MoveCamera();
            var point = mouse.position.ReadValue();
            var overPanel = point.x < 570f && point.y > Screen.height - 210f;
            if (!overPanel && mouse.leftButton.wasPressedThisFrame &&
                Physics.Raycast(view.ScreenPointToRay(point), out var hit, 30f, (1 << 7) | (1 << 30), QueryTriggerInteraction.Ignore))
                spawner.TryPlaceOrNavigate(hit.point, hit.normal);
        }
        var keyboard = Keyboard.current;
        if (keyboard == null) return;
        if (keyboard.digit1Key.wasPressedThisFrame) SetSpeed(1f);
        if (keyboard.digit4Key.wasPressedThisFrame) SetSpeed(4f);
        if (keyboard.digit8Key.wasPressedThisFrame) SetSpeed(8f);
        if (keyboard.spaceKey.wasPressedThisFrame) TogglePause();
        if (keyboard.rKey.wasPressedThisFrame) ResetPlacement(startingPoint);
    }

    void MoveCamera()
    {
        var rotation = Quaternion.Euler(pitch, yaw, 0f);
        transform.SetPositionAndRotation(orbitCenter - rotation * Vector3.forward * distance, rotation);
    }

    public void SetSpeed(float multiplier)
    {
        playingSpeed = Mathf.Clamp(multiplier, 1f, 8f);
        Time.timeScale = playingSpeed;
    }

    public void TogglePause() => Time.timeScale = Time.timeScale > 0f ? 0f : playingSpeed;

    public void ResetPlacement(Vector3 point)
    {
        spawner.ResetRobot();
        navigator = null;
        controller = null;
        spawner.TryPlaceOrNavigate(point, Vector3.up);
    }

    void OnGUI()
    {
        GUI.Box(new Rect(12, 12, 550, 195), "MicroDuck — physical navigation trial");
        GUILayout.BeginArea(new Rect(24, 38, 525, 160));
        GUILayout.Label("Click a surface: destination · Right drag: orbit · Scroll: zoom");
        GUILayout.Label($"Simulation: {Time.timeScale:0.#}× · Controller: {(controller != null ? controller.BehaviorState : "preparing")}");
        GUILayout.Label(navigator != null ? navigator.Status : "Preparing duck");
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("1×")) SetSpeed(1f);
        if (GUILayout.Button("4×")) SetSpeed(4f);
        if (GUILayout.Button("8×")) SetSpeed(8f);
        if (GUILayout.Button(Time.timeScale > 0f ? "Pause" : "Resume")) TogglePause();
        if (GUILayout.Button("Reset on table")) ResetPlacement(startingPoint);
        if (GUILayout.Button("Reset on floor")) ResetPlacement(new Vector3(-1.6f, 0f, -1f));
        GUILayout.EndHorizontal();
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Goal across room")) spawner.TryPlaceOrNavigate(new Vector3(0f, 0f, 2f), Vector3.up);
        if (GUILayout.Button("Goal behind duck") && controller != null)
        {
            var goal = controller.FootMidpoint - Vector3.ProjectOnPlane(controller.Trunk.forward, Vector3.up).normalized * 0.65f;
            if (Physics.Raycast(goal + Vector3.up * 0.3f, Vector3.down, out var hit, 2f, (1 << 7) | (1 << 30)))
                spawner.TryPlaceOrNavigate(hit.point, hit.normal);
        }
        GUILayout.EndHorizontal();
        GUILayout.EndArea();
    }

    void OnDisable()
    {
#if UNITY_EDITOR || UNITY_STANDALONE_LINUX
        Time.timeScale = previousTimeScale;
#endif
    }
}
