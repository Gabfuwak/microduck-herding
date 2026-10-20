using System;
using System.IO;
using System.Linq;
using System.Xml;
using Unity.Robotics.UrdfImporter;
using UnityEditor;
using Unity.InferenceEngine;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

public static class MicroDuckImporter
{
    public const string PrefabPath = "Assets/Robots/MicroDuck/MicroDuck.prefab";
    const string UrdfPath = "Assets/Robots/MicroDuck/microduck.urdf";
    const string NativeAssetPath = "Assets/Robots/MicroDuck/MuJoCo";
    const string StandingPolicyPath = "Assets/Robots/MicroDuck/Policies/alpha_stand.onnx";
    const string WalkingPolicyPath = "Assets/Robots/MicroDuck/Policies/alpha_walking.onnx";

    [MenuItem("Tools/MicroDuck/Rebuild MuJoCo Prefab")]
    public static void RebuildMuJoCoPrefab()
    {
        RequireEditMode();
        if (!File.Exists(UrdfPath))
            throw new FileNotFoundException("Run python3 Tools/convert_microduck.py first.", UrdfPath);
        var previousScene = SceneManager.GetActiveScene();
        var previousSelection = Selection.activeObject;
        var importScene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Additive);
        SceneManager.SetActiveScene(importScene);
        Selection.activeObject = null;
        try
        {
            // URDF is only a CAD visual importer. MuJoCo alone owns the final prefab's physics.
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
                throw new InvalidOperationException("The URDF visual importer did not create MicroDuck.");
            foreach (var behaviour in robot.GetComponentsInChildren<MonoBehaviour>(true))
                Object.DestroyImmediate(behaviour);
            foreach (var child in robot.GetComponentsInChildren<Transform>(true))
                if (child != null && child.name == "Plugins")
                    Object.DestroyImmediate(child.gameObject);
            var root = new GameObject("MicroDuck");
            robot.transform.SetParent(root.transform, false);
            robot.name = "Links";
            foreach (var child in root.GetComponentsInChildren<Transform>(true))
                child.gameObject.tag = "Untagged";
            ConfigurePolicies(root);
            foreach (var renderer in root.GetComponentsInChildren<MeshRenderer>(true))
                foreach (var material in renderer.sharedMaterials)
                {
                    if (material == null || material.shader == null || !material.shader.isSupported)
                        throw new InvalidOperationException($"Unsupported material on {renderer.name}.");
                    material.SetFloat("_Metallic", 0f);
                    material.SetFloat("_Smoothness", 0.25f);
                    EditorUtility.SetDirty(material);
                    AssetDatabase.SaveAssetIfDirty(material);
                }
            Save(root);
        }
        finally
        {
            SceneManager.SetActiveScene(previousScene);
            EditorSceneManager.CloseScene(importScene, true);
            Selection.activeObject = previousSelection;
        }
    }

    [MenuItem("Tools/MicroDuck/Configure Locomotion Prefab")]
    public static void ConfigureLocomotionPrefab()
    {
        RequireEditMode();
        var root = PrefabUtility.LoadPrefabContents(PrefabPath);
        try
        {
            ConfigurePolicies(root);
            Save(root);
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(root);
        }
    }

    static void RequireEditMode()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
            throw new InvalidOperationException("Stop Play mode before importing MicroDuck.");
    }

    static void Save(GameObject root)
    {
        if (PrefabUtility.SaveAsPrefabAsset(root, PrefabPath) == null)
            throw new InvalidOperationException("Failed to save MicroDuck MuJoCo prefab.");
        Debug.Log($"MicroDuck MuJoCo prefab configured: {root.GetComponentsInChildren<MeshFilter>(true).Length} CAD parts; no PhysX bodies or colliders. {PrefabPath}");
    }

    static void ConfigurePolicies(GameObject root)
    {
        var standing = AssetDatabase.LoadAssetAtPath<ModelAsset>(StandingPolicyPath);
        var walking = AssetDatabase.LoadAssetAtPath<ModelAsset>(WalkingPolicyPath);
        var scene = AssetDatabase.LoadAssetAtPath<TextAsset>(NativeAssetPath + "/scene.xml");
        if (standing == null || walking == null || scene == null)
            throw new InvalidOperationException("Import the policies and run python3 Tools/setup_mujoco_unity.py first.");
        // Child joints must be removed before their parent articulations.
        var articulations = root.GetComponentsInChildren<ArticulationBody>(true);
        for (var i = articulations.Length - 1; i >= 0; --i)
            Object.DestroyImmediate(articulations[i]);
        foreach (var collider in root.GetComponentsInChildren<Collider>(true))
            Object.DestroyImmediate(collider);
        foreach (var body in root.GetComponentsInChildren<Rigidbody>(true))
            Object.DestroyImmediate(body);
        var controller = root.GetComponent<MicroDuckPolicyController>();
        if (controller == null)
            controller = root.AddComponent<MicroDuckPolicyController>();
        if (root.GetComponent<MicroDuckWalkDemo>() == null)
            root.AddComponent<MicroDuckWalkDemo>();
        using var serialized = new SerializedObject(controller);
        serialized.FindProperty("standingModel").objectReferenceValue = standing;
        serialized.FindProperty("walkingModel").objectReferenceValue = walking;
        serialized.FindProperty("sceneXml").objectReferenceValue = scene;
        var assets = serialized.FindProperty("nativeAssets");
        var files = Directory.GetFiles(NativeAssetPath + "/assets", "*.bytes").OrderBy(path => path, StringComparer.Ordinal).ToArray();
        assets.arraySize = files.Length;
        for (var i = 0; i < files.Length; ++i)
        {
            var entry = assets.GetArrayElementAtIndex(i);
            entry.FindPropertyRelative("filename").stringValue = "assets/" + Path.GetFileNameWithoutExtension(files[i]);
            var asset = AssetDatabase.LoadAssetAtPath<TextAsset>(files[i]);
            if (asset == null)
                throw new InvalidOperationException($"Missing MuJoCo mesh bytes: {files[i]}");
            entry.FindPropertyRelative("data").objectReferenceValue = asset;
        }
        var xml = new XmlDocument();
        xml.LoadXml(scene.text);
        var nativeBodies = xml.SelectNodes("/mujoco/worldbody//body");
        var bodies = serialized.FindProperty("bodies");
        bodies.arraySize = nativeBodies.Count;
        for (var i = 0; i < nativeBodies.Count; ++i)
        {
            var name = nativeBodies[i].Attributes["name"].Value;
            var path = name;
            for (var parent = nativeBodies[i].ParentNode; parent.Name == "body"; parent = parent.ParentNode)
                path = parent.Attributes["name"].Value + "/" + path;
            var link = root.transform.Find("Links/" + path);
            if (link == null)
                throw new InvalidOperationException($"Missing CAD link hierarchy: Links/{path}.");
            var entry = bodies.GetArrayElementAtIndex(i);
            entry.FindPropertyRelative("name").stringValue = name;
            entry.FindPropertyRelative("transform").objectReferenceValue = link;
        }
        serialized.ApplyModifiedPropertiesWithoutUndo();
        // Preview the real native initial configuration, not a separately maintained posed skeleton.
        try
        {
            controller.InitializeSimulation();
        }
        finally
        {
            controller.ReleaseSimulation();
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
        foreach (var stl in stls)
            AssetDatabase.ImportAsset(StlAssetPostProcessor.GetPrefabAssetPath(stl),
                ImportAssetOptions.ForceUpdate | ImportAssetOptions.ForceSynchronousImport);
    }
}
