using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

public static class MicroDuckObstacleScene
{
    public const string ScenePath = "Assets/Scenes/MicroDuckObstacles.unity";
    const string MaterialsPath = "Assets/Scenes/MicroDuckObstacles/Materials";

    [MenuItem("Tools/MicroDuck/Create Box Obstacle Scene")]
    public static void Create()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
            throw new InvalidOperationException("Stop Play mode before creating the scene.");
        if (File.Exists(ScenePath))
        {
            EditorSceneManager.OpenScene(ScenePath);
            var course = Object.FindAnyObjectByType<MicroDuckObstacleCourse>();
            if (course != null) ConfigureRecoveryPolicy(course.duck);
            EditorSceneManager.SaveScene(SceneManager.GetActiveScene());
            return;
        }
        var previous = SceneManager.GetActiveScene();
        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Additive);
        SceneManager.SetActiveScene(scene);
        try
        {
            Directory.CreateDirectory(MaterialsPath);
            AssetDatabase.Refresh();
            var ground = Material("Ground", new Color(0.19f, 0.25f, 0.29f));
            var start = Material("Start", new Color(0.95f, 0.59f, 0.23f));
            var steps = Material("Steps", new Color(0.72f, 0.65f, 0.49f));
            var coral = Material("Obstacles Coral", new Color(0.75f, 0.33f, 0.27f));
            var teal = Material("Obstacles Teal", new Color(0.23f, 0.49f, 0.49f));
            var course = new GameObject("Box Course");
            Box("Ground", new Vector3(0, -0.04f, 0.3f), new Vector3(4f, 0.08f, 5f), ground, course.transform);
            Box("Starting Box", new Vector3(0, 0.15f, -1f), new Vector3(0.9f, 0.3f, 0.8f), start, course.transform);
            Box("Step 1", new Vector3(0, 0.11f, -0.35f), new Vector3(0.9f, 0.22f, 0.5f), steps, course.transform);
            Box("Step 2", new Vector3(0, 0.07f, 0.15f), new Vector3(0.9f, 0.14f, 0.5f), steps, course.transform);
            Box("Step 3", new Vector3(0, 0.03f, 0.65f), new Vector3(0.9f, 0.06f, 0.5f), steps, course.transform);
            Box("Left Tower", new Vector3(-0.85f, 0.24f, -0.1f), new Vector3(0.35f, 0.48f, 0.55f), coral, course.transform);
            Box("Right Tower", new Vector3(0.85f, 0.325f, 0.4f), new Vector3(0.4f, 0.65f, 0.4f), teal, course.transform);
            Box("Left Block", new Vector3(-0.55f, 0.125f, 1.25f), new Vector3(0.5f, 0.25f, 0.4f), teal, course.transform);
            Box("Right Block", new Vector3(0.65f, 0.09f, 1.5f), new Vector3(0.5f, 0.18f, 0.65f), coral, course.transform);
            Box("End Barrier", new Vector3(0, 0.15f, 2.05f), new Vector3(1.2f, 0.3f, 0.25f), coral, course.transform);

            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(MicroDuckImporter.PrefabPath);
            if (prefab == null) throw new InvalidOperationException("Missing MicroDuck prefab.");
            var robot = (GameObject)PrefabUtility.InstantiatePrefab(prefab, scene);
            robot.name = "MicroDuck";
            robot.transform.SetPositionAndRotation(new Vector3(0, 0.3f, -1f), Quaternion.identity);
            var joystick = robot.GetComponent<MicroDuckJoystick>();
            if (joystick != null)
            {
                joystick.enabled = false;
                PrefabUtility.RecordPrefabInstancePropertyModifications(joystick);
            }

            ConfigureRecoveryPolicy(robot.GetComponent<MicroDuckPolicyController>());
            robot.SetActive(true);
            PrefabUtility.RecordPrefabInstancePropertyModifications(robot);

            var camera = new GameObject("Course Camera").AddComponent<Camera>();
            camera.tag = "MainCamera";
            camera.nearClipPlane = 0.01f;
            camera.farClipPlane = 60;
            camera.fieldOfView = 48;
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = new Color(0.065f, 0.09f, 0.12f);
            camera.gameObject.AddComponent<AudioListener>();
            var controls = new GameObject("Course Controls").AddComponent<MicroDuckObstacleCourse>();
            controls.duck = robot.GetComponent<MicroDuckPolicyController>();
            controls.trunk = robot.GetComponentsInChildren<Transform>(true).First(t => t.name == "trunk_base");
            controls.view = camera;
            var target = new Vector3(0, 0.46f, -0.7f);
            camera.transform.position = target + Quaternion.Euler(35, -35, 0) * (Vector3.back * 2.6f);
            camera.transform.LookAt(target);
            var sun = new GameObject("Sun").AddComponent<Light>();
            sun.type = LightType.Directional;
            sun.intensity = 2;
            sun.shadows = LightShadows.Soft;
            sun.transform.rotation = Quaternion.Euler(48, -30, 0);
            RenderSettings.ambientMode = AmbientMode.Flat;
            RenderSettings.ambientLight = new Color(0.6f, 0.65f, 0.7f);
            EditorSceneManager.SaveScene(scene, ScenePath);
            AssetDatabase.SaveAssets();
        }
        finally
        {
            SceneManager.SetActiveScene(previous);
            EditorSceneManager.CloseScene(scene, true);
        }
        EditorSceneManager.OpenScene(ScenePath);
        SceneView.lastActiveSceneView?.LookAt(new Vector3(0, 0.3f, 0.2f), Quaternion.Euler(40, -30, 0), 3f);
        Debug.Log("MicroDuck obstacle course ready. Press Play; W/S move, A/D turn, Q/E sidestep, fall recovery preserves physical state.");
    }

    public static void ConfigureRecoveryPolicy(MicroDuckPolicyController controller)
    {
        var model = AssetDatabase.LoadAssetAtPath<Unity.InferenceEngine.ModelAsset>(
            "Assets/Robots/MicroDuck/Policies/velstand.onnx");
        if (model == null) throw new InvalidOperationException("Import the official velstand policy first.");
        var serialized = new SerializedObject(controller);
        serialized.FindProperty("standingModel").objectReferenceValue = model;
        serialized.FindProperty("walkingModel").objectReferenceValue = model;
        serialized.FindProperty("useRobotd").boolValue = true;
        serialized.FindProperty("robotdLockstep").boolValue = true;
        serialized.ApplyModifiedPropertiesWithoutUndo();
        PrefabUtility.RecordPrefabInstancePropertyModifications(controller);
    }

    static Material Material(string name, Color color)
    {
        var path = MaterialsPath + "/" + name + ".mat";
        var material = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (material != null) return material;
        material = new Material(Shader.Find("Universal Render Pipeline/Lit")) { name = name, color = color };
        material.SetFloat("_Smoothness", 0.3f);
        AssetDatabase.CreateAsset(material, path);
        return material;
    }

    static void Box(string name, Vector3 position, Vector3 size, Material material, Transform parent)
    {
        var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
        box.name = name;
        box.layer = 7; // MicroDuckSurfaces imports this layer into native MuJoCo.
        box.transform.SetParent(parent, false);
        box.transform.position = position;
        box.transform.localScale = size;
        box.GetComponent<Renderer>().sharedMaterial = material;
    }
}
