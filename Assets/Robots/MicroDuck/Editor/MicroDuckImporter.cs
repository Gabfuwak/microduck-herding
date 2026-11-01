using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Xml;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

public static class MicroDuckImporter
{
    public const string PrefabPath = "Assets/Robots/MicroDuck/MicroDuck.prefab";
    const string SourcePath = "External/microduck_rl/src/mjlab_microduck/robot/microduck";
    const string SourceMjcfPath = SourcePath + "/robot_allcollisions.xml";
    const string NativeAssetPath = "Assets/Robots/MicroDuck/MuJoCo";
    const string MeshAssetPath = "Assets/Robots/MicroDuck/Meshes";
    const string MaterialAssetPath = "Assets/Robots/MicroDuck/Materials";
    const string PollenPolicyPath = "Assets/Robots/MicroDuck/Policies/Pollen";

    [MenuItem("Tools/MicroDuck/Rebuild MuJoCo Prefab")]
    public static void RebuildMuJoCoPrefab()
    {
        RequireEditMode();
        if (!File.Exists(SourceMjcfPath))
            throw new FileNotFoundException("The pinned MicroDuck visual MJCF is missing.", SourceMjcfPath);

        var importScene = EditorSceneManager.NewPreviewScene();
        try
        {
            var document = new XmlDocument();
            document.Load(SourceMjcfPath);
            var root = BuildVisualHierarchy(document);
            SceneManager.MoveGameObjectToScene(root, importScene);
            try
            {
                ConfigurePolicies(root);
                Save(root);
            }
            finally
            {
                Object.DestroyImmediate(root);
            }
        }
        finally
        {
            EditorSceneManager.ClosePreviewScene(importScene);
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

    static GameObject BuildVisualHierarchy(XmlDocument document)
    {
        var worldbody = document.SelectSingleNode("/mujoco/worldbody");
        var asset = document.SelectSingleNode("/mujoco/asset");
        if (worldbody == null || asset == null)
            throw new InvalidDataException("The pinned MJCF must contain worldbody and asset sections.");

        var meshes = new Dictionary<string, Mesh[]>(StringComparer.Ordinal);
        foreach (XmlNode meshNode in asset.SelectNodes("mesh"))
        {
            var file = RequiredAttribute(meshNode, "file");
            foreach (var name in new[] { "scale", "refpos", "refquat" })
                if (meshNode.Attributes[name] != null)
                    throw new InvalidDataException($"Unsupported MJCF mesh {name} on '{file}'.");
            if (!file.EndsWith(".stl", StringComparison.OrdinalIgnoreCase) ||
                file.IndexOfAny(new[] { '/', '\\' }) >= 0 ||
                file == "." || file == "..")
                throw new InvalidDataException($"Unsupported MJCF mesh file '{file}'.");
            var stem = Path.GetFileNameWithoutExtension(file);
            if (meshes.ContainsKey(stem))
                throw new InvalidDataException($"Duplicate MJCF mesh asset '{stem}'.");
            meshes.Add(stem, LoadAndSaveMeshes(stem, Path.Combine(SourcePath, "assets", file)));
        }

        var materials = new Dictionary<string, Material>(StringComparer.Ordinal);
        foreach (XmlNode materialNode in asset.SelectNodes("material"))
        {
            var name = RequiredAttribute(materialNode, "name");
            var rgba = ParseFloats(RequiredAttribute(materialNode, "rgba"), 4, $"material {name} rgba");
            materials.Add(name, LoadAndUpdateMaterial(name, new Color(rgba[0], rgba[1], rgba[2], rgba[3])));
        }

        var root = new GameObject("MicroDuck");
        try
        {
            root.tag = "Untagged";
            var links = new GameObject("Links");
            links.transform.SetParent(root.transform, false);
            foreach (XmlNode body in worldbody.SelectNodes("body"))
                BuildBody(body, links.transform, meshes, materials);
            return root;
        }
        catch
        {
            Object.DestroyImmediate(root);
            throw;
        }
    }

    static void BuildBody(XmlNode bodyNode, Transform parent, Dictionary<string, Mesh[]> meshes,
        Dictionary<string, Material> materials)
    {
        var name = RequiredAttribute(bodyNode, "name");
        ValidatePlacementAttributes(bodyNode, "body " + name);
        var body = new GameObject(name);
        body.tag = "Untagged";
        body.transform.SetParent(parent, false);
        body.transform.localPosition = ParseVector(bodyNode.Attributes["pos"]?.Value, Vector3.zero, "body " + name + " pos");
        body.transform.localRotation = ParseQuaternion(bodyNode.Attributes["quat"]?.Value, Quaternion.identity, "body " + name + " quat");

        foreach (XmlNode geom in bodyNode.SelectNodes("geom"))
            if (IsVisualGeom(geom))
                BuildVisualGeom(geom, body.transform, meshes, materials);
        foreach (XmlNode child in bodyNode.SelectNodes("body"))
            BuildBody(child, body.transform, meshes, materials);
    }

    static bool IsVisualGeom(XmlNode geom)
    {
        var geomClass = geom.Attributes["class"]?.Value;
        if (geomClass != "visual")
            return false;
        if (geom.Attributes["type"]?.Value != "mesh")
            throw new InvalidDataException("Unsupported visual geom type; the importer only supports visual mesh geoms.");
        return true;
    }

    static void BuildVisualGeom(XmlNode geom, Transform parent, Dictionary<string, Mesh[]> meshes,
        Dictionary<string, Material> materials)
    {
        var meshName = RequiredAttribute(geom, "mesh");
        var materialName = RequiredAttribute(geom, "material");
        ValidatePlacementAttributes(geom, "visual geom " + meshName);
        if (!meshes.TryGetValue(meshName, out var meshChunks))
            throw new InvalidDataException($"Visual geom references undeclared mesh '{meshName}'.");
        if (!materials.TryGetValue(materialName, out var material))
            throw new InvalidDataException($"Visual geom references undeclared material '{materialName}'.");

        var position = ParseVector(geom.Attributes["pos"]?.Value, Vector3.zero, $"visual geom {meshName} pos");
        var rotation = ParseQuaternion(geom.Attributes["quat"]?.Value, Quaternion.identity, $"visual geom {meshName} quat");
        for (var i = 0; i < meshChunks.Length; ++i)
        {
            var part = new GameObject(meshName + "_" + i);
            part.tag = "Untagged";
            part.transform.SetParent(parent, false);
            part.transform.localPosition = position;
            part.transform.localRotation = rotation;
            part.AddComponent<MeshFilter>().sharedMesh = meshChunks[i];
            part.AddComponent<MeshRenderer>().sharedMaterial = material;
        }
    }

    static void ValidatePlacementAttributes(XmlNode node, string context)
    {
        foreach (var name in new[] { "euler", "axisangle", "xyaxes", "zaxis", "fromto" })
            if (node.Attributes[name] != null)
                throw new InvalidDataException($"Unsupported MJCF {name} placement on {context}.");
        var scale = node.Attributes["scale"];
        if (scale != null)
        {
            var values = ParseFloats(scale.Value, 3, context + " scale");
            if (values[0] != 1f || values[1] != 1f || values[2] != 1f)
                throw new InvalidDataException($"Non-unit MJCF scale is unsupported on {context}.");
        }
    }

    static Mesh[] LoadAndSaveMeshes(string stem, string sourcePath)
    {
        if (!File.Exists(sourcePath))
            throw new FileNotFoundException($"Missing pinned MicroDuck visual STL '{stem}'.", sourcePath);
        using var stream = File.OpenRead(sourcePath);
        using var reader = new BinaryReader(stream);
        if (stream.Length < 84)
            throw new InvalidDataException($"Binary STL '{sourcePath}' has a truncated header.");
        reader.ReadBytes(80);
        var facetCount = reader.ReadUInt32();
        if (stream.Length != 84L + 50L * facetCount)
            throw new InvalidDataException($"Binary STL '{sourcePath}' length does not match its facet count.");
        if (facetCount == 0)
            throw new InvalidDataException($"Binary STL '{sourcePath}' has no facets.");

        var maxFacetsPerMesh = 65535 / 3;
        var chunkCount = (int)((facetCount + maxFacetsPerMesh - 1) / maxFacetsPerMesh);
        var result = new Mesh[chunkCount];
        for (var chunk = 0; chunk < chunkCount; ++chunk)
        {
            var facetsInChunk = (int)Math.Min(maxFacetsPerMesh, facetCount - (uint)(chunk * maxFacetsPerMesh));
            var vertexCount = facetsInChunk * 3;
            var vertices = new Vector3[vertexCount];
            var normals = new Vector3[vertexCount];
            var triangles = new int[vertexCount];
            for (var facet = 0; facet < facetsInChunk; ++facet)
            {
                var normal = RosToUnity(ReadStlVector(reader));
                var v0 = RosToUnity(ReadStlVector(reader));
                var v1 = RosToUnity(ReadStlVector(reader));
                var v2 = RosToUnity(ReadStlVector(reader));
                reader.ReadUInt16();
                var first = facet * 3;
                vertices[first] = v0;
                vertices[first + 1] = v2;
                vertices[first + 2] = v1;
                for (var vertex = 0; vertex < 3; ++vertex)
                {
                    normals[first + vertex] = normal;
                    triangles[first + vertex] = first + vertex;
                }
            }
            var mesh = new Mesh { name = stem + "_" + chunk };
            mesh.vertices = vertices;
            mesh.normals = normals;
            mesh.triangles = triangles;
            var path = MeshAssetPath + "/" + mesh.name + ".asset";
            var existing = AssetDatabase.LoadAssetAtPath<Mesh>(path);
            if (existing == null)
                AssetDatabase.CreateAsset(mesh, path);
            else
            {
                EditorUtility.CopySerialized(mesh, existing);
                Object.DestroyImmediate(mesh);
                EditorUtility.SetDirty(existing);
                mesh = existing;
            }
            AssetDatabase.SaveAssetIfDirty(mesh);
            result[chunk] = mesh;
        }
        return result;
    }

    static Vector3 ReadStlVector(BinaryReader reader)
    {
        return new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
    }

    static Vector3 RosToUnity(Vector3 value) => new Vector3(-value.y, value.z, value.x);

    static Material LoadAndUpdateMaterial(string name, Color color)
    {
        var path = MaterialAssetPath + "/" + name + ".mat";
        var material = AssetDatabase.LoadAssetAtPath<Material>(path);
        var isNew = material == null;
        var shader = Shader.Find("Universal Render Pipeline/Lit");
        if (shader == null)
            throw new InvalidOperationException("Universal Render Pipeline/Lit shader is unavailable.");
        if (isNew)
            material = new Material(shader) { name = name };
        material.shader = shader;
        material.SetColor("_BaseColor", color);
        material.SetColor("_Color", color);
        material.SetFloat("_Metallic", 0f);
        material.SetFloat("_Smoothness", 0.25f);
        if (isNew)
            AssetDatabase.CreateAsset(material, path);
        EditorUtility.SetDirty(material);
        AssetDatabase.SaveAssetIfDirty(material);
        return material;
    }

    static string RequiredAttribute(XmlNode node, string name)
    {
        var value = node.Attributes[name]?.Value;
        if (string.IsNullOrEmpty(value))
            throw new InvalidDataException($"MJCF {node.Name} is missing required '{name}' attribute.");
        return value;
    }

    static Vector3 ParseVector(string value, Vector3 fallback, string context)
    {
        if (value == null)
            return fallback;
        var values = ParseFloats(value, 3, context);
        return RosToUnity(new Vector3(values[0], values[1], values[2]));
    }

    static Quaternion ParseQuaternion(string value, Quaternion fallback, string context)
    {
        if (value == null)
            return fallback;
        var values = ParseFloats(value, 4, context);
        var quaternion = new Quaternion(values[2], -values[3], -values[1], values[0]);
        var magnitude = Mathf.Sqrt(quaternion.x * quaternion.x + quaternion.y * quaternion.y +
                                   quaternion.z * quaternion.z + quaternion.w * quaternion.w);
        if (magnitude < 1e-8f)
            throw new InvalidDataException($"MJCF {context} quaternion has zero magnitude.");
        quaternion.x /= magnitude;
        quaternion.y /= magnitude;
        quaternion.z /= magnitude;
        quaternion.w /= magnitude;
        return quaternion;
    }

    static float[] ParseFloats(string value, int count, string context)
    {
        var tokens = value.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
        if (tokens.Length != count)
            throw new InvalidDataException($"MJCF {context} must contain {count} numbers.");
        var result = new float[count];
        for (var i = 0; i < count; ++i)
        {
            result[i] = float.Parse(tokens[i], NumberStyles.Float, CultureInfo.InvariantCulture);
            if (float.IsNaN(result[i]) || float.IsInfinity(result[i]))
                throw new InvalidDataException($"MJCF {context} contains a non-finite number.");
        }
        return result;
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
        Debug.Log($"MicroDuck MuJoCo prefab configured: {root.GetComponentsInChildren<MeshFilter>(true).Length} visual parts; no PhysX bodies or colliders. {PrefabPath}");
    }

    static void ConfigurePolicies(GameObject root)
    {
        var policies = Directory.GetFiles(PollenPolicyPath, "*.bytes").OrderBy(path => path, StringComparer.Ordinal)
            .Select(AssetDatabase.LoadAssetAtPath<TextAsset>).ToArray();
        var scene = AssetDatabase.LoadAssetAtPath<TextAsset>(NativeAssetPath + "/scene.xml");
        if (policies.Length == 0 || policies.Any(policy => policy == null) || scene == null)
            throw new InvalidOperationException("The packaged Pollen policies and MuJoCo scene assets must be present.");
        var controller = root.GetComponent<MicroDuckPolicyController>();
        if (controller == null)
            controller = root.AddComponent<MicroDuckPolicyController>();
        if (root.GetComponent<MicroDuckJoystick>() == null)
            root.AddComponent<MicroDuckJoystick>();
        if (root.GetComponent<MicroDuckNavigator>() == null)
            root.AddComponent<MicroDuckNavigator>();
        using var serialized = new SerializedObject(controller);
        var policyArray = serialized.FindProperty("controlPolicies");
        policyArray.arraySize = policies.Length;
        for (var i = 0; i < policies.Length; i++)
            policyArray.GetArrayElementAtIndex(i).objectReferenceValue = policies[i];
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
                throw new InvalidOperationException($"Missing visual link hierarchy: Links/{path}.");
            var entry = bodies.GetArrayElementAtIndex(i);
            entry.FindPropertyRelative("name").stringValue = name;
            entry.FindPropertyRelative("transform").objectReferenceValue = link;
        }
        serialized.ApplyModifiedPropertiesWithoutUndo();
        try
        {
            controller.InitializeSimulation();
        }
        finally
        {
            controller.ReleaseSimulation();
        }
    }
}
