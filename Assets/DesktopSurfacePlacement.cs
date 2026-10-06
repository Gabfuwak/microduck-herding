using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;
using UnityEngine.XR.Interaction.Toolkit.Samples.StarterAssets;

public sealed class DesktopSurfacePlacement : MonoBehaviour
{
    [SerializeField] private Camera placementCamera;
    [SerializeField] private ObjectSpawner objectSpawner;
    [SerializeField] private LayerMask surfaceLayers;
    [SerializeField] private Transform surfaceDot;
    private readonly Dictionary<Scene, GameObject> simulationColliders = new();

    private void LateUpdate()
    {
        Ray ray = placementCamera.ViewportPointToRay(new Vector3(0.5f, 0.5f, 0f));
        bool hasHit = Physics.Raycast(ray, out RaycastHit hit, Mathf.Infinity, surfaceLayers, QueryTriggerInteraction.Ignore);
        float distance = hasHit ? hit.distance : Mathf.Infinity;
        for (int i = 0; i < SceneManager.sceneCount; i++)
        {
            Scene scene = SceneManager.GetSceneAt(i);
            if (simulationColliders.ContainsKey(scene))
                continue;
            PhysicsScene physicsScene = scene.GetPhysicsScene();
            if (physicsScene == Physics.defaultPhysicsScene)
                continue;

            if (physicsScene.Raycast(ray.origin, ray.direction, out RaycastHit sceneHit, distance, surfaceLayers, QueryTriggerInteraction.Ignore))
            {
                hit = sceneHit;
                distance = hit.distance;
                hasHit = true;
                MirrorSimulationColliders(scene);
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

    private void MirrorSimulationColliders(Scene scene)
    {
        if (simulationColliders.ContainsKey(scene))
            return;

        // XR Simulation scans a separate physics world; the gameplay articulation needs a copy.
        var root = new GameObject("Desktop Simulation Colliders");
        SceneManager.MoveGameObjectToScene(root, gameObject.scene);
        simulationColliders.Add(scene, root);
        foreach (var sceneRoot in scene.GetRootGameObjects())
        {
            foreach (var source in sceneRoot.GetComponentsInChildren<MeshCollider>())
            {
                if (!source.enabled || !source.gameObject.activeInHierarchy || source.isTrigger ||
                    (surfaceLayers.value & (1 << source.gameObject.layer)) == 0)
                    continue;
                var surface = new GameObject(source.name) { layer = source.gameObject.layer };
                surface.transform.SetParent(root.transform, false);
                surface.transform.SetPositionAndRotation(source.transform.position, source.transform.rotation);
                surface.transform.localScale = source.transform.lossyScale;
                var collider = surface.AddComponent<MeshCollider>();
                collider.sharedMesh = source.sharedMesh;
                collider.convex = source.convex;
                collider.sharedMaterial = source.sharedMaterial;
                collider.contactOffset = 0.001f;
            }
        }
        Physics.SyncTransforms();
    }

    private void OnDisable()
    {
        if (surfaceDot != null)
            surfaceDot.gameObject.SetActive(false);
        foreach (var root in simulationColliders.Values)
            Destroy(root);
        simulationColliders.Clear();
    }
}
