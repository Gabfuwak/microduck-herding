using System;
using System.Collections.Generic;
using System.IO;
using Unity.Robotics.UrdfImporter;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

public static class MicroDuckImporter
{
    public const string PrefabPath = "Assets/Robots/MicroDuck/MicroDuck.prefab";
    const string UrdfPath = "Assets/Robots/MicroDuck/microduck.urdf";

    [InitializeOnLoadMethod]
    static void Initialize() => EditorApplication.delayCall += BakeEditorCollisions;

    static void BakeEditorCollisions()
    {
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
        if (prefab == null)
            return;
        var baked = new HashSet<Mesh>();
        foreach (var collider in prefab.GetComponentsInChildren<MeshCollider>(true))
            if (baked.Add(collider.sharedMesh))
                Physics.BakeMesh(collider.sharedMesh.GetEntityId(), true, collider.cookingOptions);
    }

    [MenuItem("Tools/MicroDuck/Rebuild Physics Prefab")]
    public static void RebuildPhysicsPrefab()
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
            PrepareMeshes();
            GameObject robot = null;
            using (var importer = UrdfRobotExtensions.Create(Path.GetFullPath(UrdfPath), settings))
            {
                while (importer.MoveNext())
                    if (importer.Current != null)
                        robot = importer.Current;
            }
            if (robot == null)
                throw new InvalidOperationException("The URDF importer did not create MicroDuck.");

            ConfigurePhysics(robot);
            foreach (var behaviour in robot.GetComponentsInChildren<MonoBehaviour>(true))
                Object.DestroyImmediate(behaviour);
            foreach (var child in robot.GetComponentsInChildren<Transform>(true))
                if (child != null && child.name == "Plugins")
                    Object.DestroyImmediate(child.gameObject);

            var root = new GameObject("MicroDuck");
            robot.transform.SetParent(root.transform, false);
            robot.name = "Links";
            robot.transform.localPosition = Vector3.zero;
            robot.transform.localRotation = Quaternion.identity;
            foreach (var child in root.GetComponentsInChildren<Transform>(true))
                child.gameObject.tag = "Untagged";

            // Ground both geometry sets, with 2 mm clearance for contact generation.
            var minimumY = float.PositiveInfinity;
            var filters = root.GetComponentsInChildren<MeshFilter>(true);
            foreach (var filter in filters)
            {
                if (filter.sharedMesh == null)
                    throw new InvalidOperationException($"Missing mesh on {filter.name}.");
                foreach (var vertex in filter.sharedMesh.vertices)
                    minimumY = Mathf.Min(minimumY, filter.transform.TransformPoint(vertex).y);
            }
            foreach (var collider in root.GetComponentsInChildren<MeshCollider>(true))
                foreach (var vertex in collider.sharedMesh.vertices)
                    minimumY = Mathf.Min(minimumY, collider.transform.TransformPoint(vertex).y);
            if (float.IsInfinity(minimumY))
                throw new InvalidOperationException("MicroDuck contains no visual vertices.");
            robot.transform.localPosition = new Vector3(0f, -minimumY + 0.002f, 0f);

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
            BakeEditorCollisions();
            Debug.Log($"MicroDuck physics import complete: {filters.Length} mesh parts, " +
                $"{root.GetComponentsInChildren<ArticulationBody>().Length} bodies, " +
                $"{root.GetComponentsInChildren<Collider>().Length} colliders. {PrefabPath}");
        }
        finally
        {
            SceneManager.SetActiveScene(previousScene);
            EditorSceneManager.CloseScene(importScene, true);
            Selection.activeObject = previousSelection;
        }
    }

    static void PrepareMeshes()
    {
        var stls = Directory.GetFiles("Assets/Robots/MicroDuck/Meshes", "*.stl");
        foreach (var stl in stls)
        {
            var meshes = StlImporter.ImportMesh(stl);
            if (meshes == null || meshes.Length == 0)
                throw new InvalidOperationException($"No mesh data in {stl}.");
            var root = new GameObject(Path.GetFileNameWithoutExtension(stl));
            try
            {
                for (var i = 0; i < meshes.Length; i++)
                {
                    var path = StlAssetPostProcessor.GetMeshAssetPath(stl, i);
                    meshes[i].name = Path.GetFileNameWithoutExtension(path);
                    var mesh = AssetDatabase.LoadAssetAtPath<Mesh>(path);
                    if (mesh == null)
                    {
                        mesh = meshes[i];
                        AssetDatabase.CreateAsset(mesh, path);
                    }
                    else
                    {
                        // Replacing a loaded mesh asset invalidates references in cached STL prefabs.
                        EditorUtility.CopySerialized(meshes[i], mesh);
                        Object.DestroyImmediate(meshes[i]);
                        EditorUtility.SetDirty(mesh);
                    }
                    var part = new GameObject(Path.GetFileNameWithoutExtension(path));
                    part.transform.SetParent(root.transform, false);
                    part.AddComponent<MeshFilter>().sharedMesh = mesh;
                    part.AddComponent<MeshRenderer>();
                }
                if (PrefabUtility.SaveAsPrefabAsset(root, StlAssetPostProcessor.GetPrefabAssetPath(stl)) == null)
                    throw new InvalidOperationException($"Failed to save mesh prefab for {stl}.");
            }
            finally
            {
                Object.DestroyImmediate(root);
            }
        }
        AssetDatabase.SaveAssets();
        // Mesh saves refresh native objects; reload wrappers before the URDF importer reads them.
        foreach (var stl in stls)
            AssetDatabase.ImportAsset(StlAssetPostProcessor.GetPrefabAssetPath(stl),
                ImportAssetOptions.ForceUpdate | ImportAssetOptions.ForceSynchronousImport);
    }

    static void ConfigurePhysics(GameObject robot)
    {
        var source = new Robot(Path.GetFullPath(UrdfPath));
        var bodies = robot.GetComponentsInChildren<ArticulationBody>(true);
        if (bodies.Length != source.links.Count)
            throw new InvalidOperationException("MicroDuck articulation does not match the URDF links.");
        foreach (var body in bodies)
        {
            var link = source.links.Find(candidate => candidate.name == body.name);
            if (link?.inertial == null)
                throw new InvalidOperationException($"Missing source inertia for {body.name}.");
            var inertia = link.inertial.inertia;
            var matrix = new Matrix3x3(new[]
            {
                (float)inertia.ixx, (float)inertia.ixy, (float)inertia.ixz,
                (float)inertia.iyy, (float)inertia.iyz, (float)inertia.izz,
            });
            matrix.DiagonalizeRealSymmetric(out var values, out _);
            if (values.x <= 0f || values.y <= 0f || values.z <= 0f)
                throw new InvalidOperationException($"Non-positive inertia for {body.name}.");
            // The importer clamps tiny CAD inertias; keep its basis but restore source eigenvalues.
            body.automaticCenterOfMass = false;
            body.automaticInertiaTensor = false;
            body.inertiaTensor = new Vector3(values.y, values.z, values.x);
            if (body.isRoot)
                body.immovable = false;
            body.useGravity = true;
            body.linearDamping = 0f;
            body.angularDamping = 0f;
            body.jointFriction = 0f;
            body.maxAngularVelocity = 50f;
            body.solverIterations = 16;
            body.solverVelocityIterations = 4;
            body.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
            var drive = body.xDrive;
            drive.stiffness = 0f;
            drive.damping = 0f;
            drive.forceLimit = 0f;
            body.xDrive = drive;
        }
        foreach (var collider in robot.GetComponentsInChildren<Collider>(true))
        {
            if (collider is not MeshCollider mesh || mesh.sharedMesh == null)
                throw new InvalidOperationException($"Missing dynamic collision mesh on {collider.name}.");
            mesh.convex = true;
            using var asset = new SerializedObject(mesh.sharedMesh);
            asset.FindProperty("m_PreBakeConvexCollisionMesh").boolValue = true;
            asset.ApplyModifiedPropertiesWithoutUndo();
            collider.contactOffset = 0.001f;
        }
    }
}
