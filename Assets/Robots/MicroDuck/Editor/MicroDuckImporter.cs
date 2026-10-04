using System;
using System.IO;
using Unity.Robotics.UrdfImporter;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

/// <summary>
/// Builds the visual-only placement prefab from the generated MicroDuck URDF.
/// Regenerate the URDF with python3 Tools/convert_microduck.py before rebuilding.
/// Link transforms and source materials are retained; physics and demo controllers are removed.
/// The root pivot sits on the lowest visual vertex, with ROS +X facing Unity +Z.
/// </summary>
public static class MicroDuckImporter
{
    public const string PrefabPath = "Assets/Robots/MicroDuck/MicroDuck.prefab";
    const string UrdfPath = "Assets/Robots/MicroDuck/microduck.urdf";

    [MenuItem("Tools/MicroDuck/Rebuild Visual Prefab")]
    public static void RebuildVisualPrefab()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
            throw new InvalidOperationException("Stop Play mode before importing MicroDuck.");
        if (!File.Exists(UrdfPath))
            throw new FileNotFoundException("Run python3 Tools/convert_microduck.py first.", UrdfPath);

        var previousScene = SceneManager.GetActiveScene();
        var previousSelection = Selection.activeObject;
        var importScene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Additive);
        SceneManager.SetActiveScene(importScene);
        Selection.activeObject = null;
        try
        {
            // STL data and URDF frames are both ROS coordinates; no additional mesh-axis correction.
            var settings = new ImportSettings
            {
                chosenAxis = ImportSettings.axisType.yAxis,
                convexMethod = ImportSettings.convexDecomposer.unity,
                OverwriteExistingPrefabs = false,
            };
            // Generate each shared mesh once; repeated STL instances must reuse the same asset.
            foreach (var stl in Directory.GetFiles("Assets/Robots/MicroDuck/Meshes", "*.stl"))
                StlAssetPostProcessor.PostprocessStlFile(stl);
            GameObject robot = null;
            using (var importer = UrdfRobotExtensions.Create(Path.GetFullPath(UrdfPath), settings))
            {
                while (importer.MoveNext())
                    if (importer.Current != null)
                        robot = importer.Current;
            }
            if (robot == null)
                throw new InvalidOperationException("The URDF importer did not create MicroDuck.");

            // Remove behaviour dependencies before the physics components they require.
            foreach (var behaviour in robot.GetComponentsInChildren<MonoBehaviour>(true))
                Object.DestroyImmediate(behaviour);
            foreach (var collider in robot.GetComponentsInChildren<Collider>(true))
                Object.DestroyImmediate(collider);
            foreach (var body in robot.GetComponentsInChildren<ArticulationBody>(true))
                Object.DestroyImmediate(body);
            foreach (var body in robot.GetComponentsInChildren<Rigidbody>(true))
                Object.DestroyImmediate(body);
            foreach (var child in robot.GetComponentsInChildren<Transform>(true))
                if (child != null && (child.name == "Collisions" || child.name == "Plugins"))
                    Object.DestroyImmediate(child.gameObject);

            var root = new GameObject("MicroDuck");
            robot.transform.SetParent(root.transform, false);
            robot.name = "Links";
            robot.transform.localPosition = Vector3.zero;
            robot.transform.localRotation = Quaternion.identity;
            foreach (var child in root.GetComponentsInChildren<Transform>(true))
                child.gameObject.tag = "Untagged";

            // Bounds of transformed vertices, rather than rotated mesh AABBs, avoid a floating pivot.
            var minimumY = float.PositiveInfinity;
            var filters = root.GetComponentsInChildren<MeshFilter>(true);
            foreach (var filter in filters)
            {
                if (filter.sharedMesh == null)
                    throw new InvalidOperationException($"Missing mesh on {filter.name}.");
                foreach (var vertex in filter.sharedMesh.vertices)
                    minimumY = Mathf.Min(minimumY, filter.transform.TransformPoint(vertex).y);
            }
            if (float.IsInfinity(minimumY))
                throw new InvalidOperationException("MicroDuck contains no visual vertices.");
            robot.transform.localPosition = new Vector3(0f, -minimumY, 0f);

            // CAD colours are already imported; use a matte finish rather than the importer's shiny default.
            foreach (var renderer in root.GetComponentsInChildren<MeshRenderer>(true))
            {
                foreach (var material in renderer.sharedMaterials)
                {
                    if (material == null || material.shader == null || !material.shader.isSupported)
                        throw new InvalidOperationException($"Unsupported material on {renderer.name}.");
                    material.SetFloat("_Metallic", 0f);
                    material.SetFloat("_Smoothness", 0.25f);
                    EditorUtility.SetDirty(material);
                }
            }
            var prefab = PrefabUtility.SaveAsPrefabAsset(root, PrefabPath);
            if (prefab == null)
                throw new InvalidOperationException("Failed to save MicroDuck prefab.");
            AssetDatabase.SaveAssets();
            Debug.Log($"MicroDuck visual import complete: {filters.Length} mesh parts, no physics. {PrefabPath}");
        }
        finally
        {
            SceneManager.SetActiveScene(previousScene);
            EditorSceneManager.CloseScene(importScene, true);
            Selection.activeObject = previousSelection;
        }
    }
}
