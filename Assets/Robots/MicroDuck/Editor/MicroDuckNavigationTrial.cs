using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class MicroDuckNavigationTrial
{
    public const string ScenePath = "Assets/Scenes/MicroDuckNavigationTrial.unity";

    [MenuItem("Tools/MicroDuck/Open Desktop Navigation Trial")]
    public static void Open()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode) return;
        if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
        if (System.IO.File.Exists(ScenePath)) EditorSceneManager.OpenScene(ScenePath);
        else CreateAndOpen();
    }

    // Also callable by editor automation. It only writes the dedicated trial
    // scene and its own materials; it never changes MainScene or the duck prefab.
    public static void CreateAndOpen()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
            throw new System.InvalidOperationException("Stop Play mode before creating the trial scene.");
        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        scene.name = "MicroDuckNavigationTrial";
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
        RenderSettings.ambientLight = new Color(0.55f, 0.58f, 0.65f);
        var light = new GameObject("Sun").AddComponent<Light>();
        light.type = LightType.Directional;
        light.intensity = 2f;
        light.transform.rotation = Quaternion.Euler(45f, -25f, 0f);
        Box("Floor", new Vector3(0f, -0.05f, 0f), new Vector3(8f, 0.10f, 8f), new Color(0.22f, 0.28f, 0.32f));
        Box("Starting table", new Vector3(0f, 0.225f, -1f), new Vector3(1.5f, 0.45f, 1.5f), new Color(0.65f, 0.55f, 0.34f));
        Box("Tall obstacle", new Vector3(0.40f, 0.40f, 0.65f), new Vector3(0.65f, 0.80f, 0.65f), new Color(0.20f, 0.52f, 0.52f));
        Box("Low obstacle", new Vector3(-1.15f, 0.225f, 0.5f), new Vector3(0.60f, 0.45f, 0.60f), new Color(0.85f, 0.43f, 0.16f));
        Box("Far obstacle", new Vector3(1.6f, 0.30f, 1.5f), new Vector3(0.70f, 0.60f, 0.70f), new Color(0.64f, 0.27f, 0.25f));
        var camera = new GameObject("Desktop Camera").AddComponent<Camera>();
        camera.tag = "MainCamera";
        camera.nearClipPlane = 0.02f;
        camera.farClipPlane = 50f;
        camera.backgroundColor = new Color(0.12f, 0.15f, 0.19f);
        camera.clearFlags = CameraClearFlags.SolidColor;
        camera.gameObject.AddComponent<AudioListener>();
        var spawner = new GameObject("MicroDuck Spawner").AddComponent<MicroDuckSpawner>();
        var serializedSpawner = new SerializedObject(spawner);
        serializedSpawner.FindProperty("m_CameraToFace").objectReferenceValue = camera;
        serializedSpawner.FindProperty("m_ObjectPrefab").objectReferenceValue = AssetDatabase.LoadAssetAtPath<GameObject>(MicroDuckImporter.PrefabPath);
        serializedSpawner.ApplyModifiedPropertiesWithoutUndo();
        var trial = camera.gameObject.AddComponent<MicroDuckDesktopTrial>();
        var serializedTrial = new SerializedObject(trial);
        serializedTrial.FindProperty("spawner").objectReferenceValue = spawner;
        serializedTrial.ApplyModifiedPropertiesWithoutUndo();
        if (!EditorSceneManager.SaveScene(scene, ScenePath))
            throw new System.InvalidOperationException("Could not save the navigation trial scene.");
        Debug.Log("Desktop trial ready: press Play. The same duck prefab and spawner are used by the Quest scene.");
    }

    static void Box(string name, Vector3 center, Vector3 size, Color color)
    {
        var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
        box.name = name;
        box.layer = 7;
        box.transform.position = center;
        box.transform.localScale = size;
        var path = "Assets/Desktop/Trial " + name + ".mat";
        var material = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (material == null)
        {
            material = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            material.SetColor("_BaseColor", color);
            AssetDatabase.CreateAsset(material, path);
        }
        box.GetComponent<Renderer>().sharedMaterial = material;
    }
}
