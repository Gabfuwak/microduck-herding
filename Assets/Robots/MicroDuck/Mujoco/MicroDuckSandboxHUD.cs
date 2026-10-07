using UnityEngine;

public sealed class MicroDuckSandboxHUD : MonoBehaviour
{
    MicroDuckMujocoController controller;
    void Awake() => controller = GetComponent<MicroDuckMujocoController>();
    void LateUpdate()
    {
        if (Camera.main == null || controller.links == null || controller.links.Length == 0) return;
        Vector3 target = controller.links[0].position + Vector3.up * 0.08f;
        Camera.main.transform.position = target + new Vector3(0.65f, 0.45f, -0.65f);
        Camera.main.transform.LookAt(target);
    }
    void OnGUI()
    {
        GUI.Box(new Rect(12, 12, 380, 100), "MicroDuck · MuJoCo");
        GUI.Label(new Rect(24, 38, 350, 24), "Arrow keys: steer · Space: stand · R: reset");
        GUI.Label(new Rect(24, 62, 350, 24), "Automatic walking starts after 5 seconds.");
        GUI.Label(new Rect(24, 86, 350, 24), $"{controller.SimulationTime:F1}s · {(controller.IsUpright ? "Upright" : "Settling / recovering")} · {controller.Height:F3} m");
    }
}
