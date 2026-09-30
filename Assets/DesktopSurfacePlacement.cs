using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.XR.Interaction.Toolkit.Samples.StarterAssets;

public sealed class DesktopSurfacePlacement : MonoBehaviour
{
    [SerializeField] private Camera placementCamera;
    [SerializeField] private ObjectSpawner objectSpawner;
    [SerializeField] private LayerMask surfaceLayers;

    private void Update()
    {
        if (Mouse.current == null || !Mouse.current.leftButton.wasPressedThisFrame)
            return;

        Ray ray = placementCamera.ViewportPointToRay(new Vector3(0.5f, 0.5f, 0f));
        if (!Physics.Raycast(ray, out RaycastHit hit, Mathf.Infinity, surfaceLayers, QueryTriggerInteraction.Ignore))
            return;

        objectSpawner.cameraToFace = placementCamera;
        objectSpawner.TrySpawnObject(hit.point, hit.normal);
    }
}
