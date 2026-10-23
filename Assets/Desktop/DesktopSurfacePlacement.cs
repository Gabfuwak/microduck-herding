using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;

public sealed class DesktopSurfacePlacement : MonoBehaviour
{
    [SerializeField] private Camera placementCamera;
    [SerializeField] private MicroDuckSpawner objectSpawner;
    [SerializeField] private LayerMask surfaceLayers;
    [SerializeField] private Transform surfaceDot;

    private void LateUpdate()
    {
        Ray ray = placementCamera.ViewportPointToRay(new Vector3(0.5f, 0.5f, 0f));
        bool hasHit = Physics.Raycast(ray, out RaycastHit hit, Mathf.Infinity, surfaceLayers, QueryTriggerInteraction.Ignore);
        float distance = hasHit ? hit.distance : Mathf.Infinity;
        for (int i = 0; i < SceneManager.sceneCount; i++)
        {
            Scene scene = SceneManager.GetSceneAt(i);
            PhysicsScene physicsScene = scene.GetPhysicsScene();
            if (physicsScene == Physics.defaultPhysicsScene)
                continue;

            if (physicsScene.Raycast(ray.origin, ray.direction, out RaycastHit sceneHit, distance, surfaceLayers, QueryTriggerInteraction.Ignore))
            {
                hit = sceneHit;
                distance = hit.distance;
                hasHit = true;
            }
        }

        surfaceDot.gameObject.SetActive(hasHit);
        if (!hasHit)
            return;

        surfaceDot.position = hit.point + hit.normal * 0.005f;
        if (Mouse.current == null || !Mouse.current.leftButton.wasPressedThisFrame)
            return;

        objectSpawner.cameraToFace = placementCamera;
        objectSpawner.TrySpawnObject(hit.point, hit.normal);
    }


    private void OnDisable()
    {
        if (surfaceDot != null)
            surfaceDot.gameObject.SetActive(false);
    }
}
