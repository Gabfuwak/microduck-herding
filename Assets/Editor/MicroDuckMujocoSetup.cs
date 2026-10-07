using System;
using System.Linq;
using Unity.InferenceEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

public static class MicroDuckMujocoSetup
{
    const string PrefabPath = "Assets/Robots/MicroDuck/Mujoco/MicroDuckMujoco.prefab";
    const string SandboxPath = "Assets/Scenes/MicroDuckMujocoSandbox.unity";

    [MenuItem("Tools/MicroDuck/Set Up MuJoCo")]
    public static void SetUp()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
            throw new InvalidOperationException("Stop Play mode before setting up MuJoCo.");
        var source = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Robots/MicroDuck/MicroDuck.prefab");
        if (source == null) throw new InvalidOperationException("Missing existing MicroDuck mesh prefab.");
        var current = SceneManager.GetActiveScene();
        var staging = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Additive);
        SceneManager.SetActiveScene(staging);
        GameObject prefab;
        try
        {
            var robot = (GameObject)PrefabUtility.InstantiatePrefab(source, staging);
            PrefabUtility.UnpackPrefabInstance(robot, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
            robot.name = "MicroDuck MuJoCo";
            robot.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            var links = robot.GetComponentsInChildren<ArticulationBody>(true).Select(b => b.transform).ToArray();
            foreach (var component in robot.GetComponentsInChildren<Component>(true).Reverse())
                if (component != null && !(component is Transform) && !(component is MeshFilter) && !(component is MeshRenderer))
                    Object.DestroyImmediate(component);
            var controller = robot.AddComponent<MicroDuckMujocoController>();
            controller.links = links;
            controller.walkingModel = AssetDatabase.LoadAssetAtPath<ModelAsset>("Assets/Robots/MicroDuck/Mujoco/BEST_alpha_walking.onnx");
            controller.recoveryModel = AssetDatabase.LoadAssetAtPath<ModelAsset>("Assets/Robots/MicroDuck/Mujoco/BEST_alpha_stand.onnx");
            if (controller.walkingModel == null || controller.recoveryModel == null || links.Length != 15)
                throw new InvalidOperationException("Missing policies or robot links.");
            prefab = PrefabUtility.SaveAsPrefabAsset(robot, PrefabPath);
            if (prefab == null) throw new InvalidOperationException("Could not save MuJoCo prefab.");
            Object.DestroyImmediate(robot);
            BuildSandbox(prefab);
            EditorSceneManager.SaveScene(staging, SandboxPath);
        }
        finally
        {
            SceneManager.SetActiveScene(current);
            EditorSceneManager.CloseScene(staging, true);
        }
        // Replace only the robot choice in the current project's placement flow.
        foreach (var root in current.GetRootGameObjects())
            foreach (var behaviour in root.GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (behaviour == null || behaviour.GetType().Name != "ObjectSpawner") continue;
                using var serialized = new SerializedObject(behaviour);
                var choices = serialized.FindProperty("m_ObjectPrefabs");
                if (choices == null) continue;
                for (int i = 0; i < choices.arraySize; i++)
                {
                    var choice = choices.GetArrayElementAtIndex(i);
                    if (choice.objectReferenceValue == source)
                        choice.objectReferenceValue = prefab;
                }
                serialized.ApplyModifiedProperties();
            }
        if (current.path == "Assets/Scenes/MainScene.unity") EditorSceneManager.SaveScene(current);
        AssetDatabase.SaveAssets();
        Debug.Log("MuJoCo ready. MainScene spawns native MuJoCo ducks. Open Tools/MicroDuck/Open MuJoCo Sandbox to test.");
    }

    static void BuildSandbox(GameObject prefab)
    {
        var floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
        floor.name = "MuJoCo floor visual";
        floor.transform.localScale = new Vector3(0.6f, 1, 0.6f);
        var floorMaterial = AssetDatabase.LoadAssetAtPath<Material>("Assets/Robots/MicroDuck/Mujoco/SandboxFloor.mat");
        if (floorMaterial == null)
        {
            floorMaterial = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            floorMaterial.color = new Color(0.24f, 0.31f, 0.33f);
            AssetDatabase.CreateAsset(floorMaterial, "Assets/Robots/MicroDuck/Mujoco/SandboxFloor.mat");
        }
        floor.GetComponent<Renderer>().sharedMaterial = floorMaterial;
        Object.DestroyImmediate(floor.GetComponent<Collider>());
        var robot = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
        robot.GetComponent<MicroDuckMujocoController>().keyboardControl = true;
        var camera = new GameObject("Sandbox Camera").AddComponent<Camera>();
        camera.tag = "MainCamera";
        camera.transform.position = new Vector3(0.85f, 0.65f, -0.85f);
        camera.transform.LookAt(new Vector3(0, 0.14f, 0.25f));
        camera.nearClipPlane = 0.01f;
        camera.fieldOfView = 45;
        camera.backgroundColor = new Color(0.12f, 0.16f, 0.19f);
        camera.clearFlags = CameraClearFlags.SolidColor;
        camera.gameObject.AddComponent<AudioListener>();
        var light = new GameObject("Sandbox Light").AddComponent<Light>();
        light.type = LightType.Directional;
        light.intensity = 2;
        light.transform.rotation = Quaternion.Euler(45, -30, 0);
        RenderSettings.ambientLight = new Color(0.55f, 0.55f, 0.55f);
        robot.AddComponent<MicroDuckSandboxHUD>();
    }

    [MenuItem("Tools/MicroDuck/Open MuJoCo Sandbox")]
    public static void OpenSandbox()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
            throw new InvalidOperationException("Stop Play mode first.");
        EditorSceneManager.OpenScene(SandboxPath);
    }
}
