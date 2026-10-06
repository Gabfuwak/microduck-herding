using System;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;

// The URDF package's desktop Assimp libraries are editor-only, despite its Any Platform defaults.
public sealed class UrdfEditorPluginFilter : IPreprocessBuildWithReport
{
    const string NativePluginRoot = "Packages/com.unity.robotics.urdf-importer/Runtime/UnityMeshImporter/Plugins/AssimpNet/Native/";

    public int callbackOrder => 0;

    [InitializeOnLoadMethod]
    static void Initialize()
    {
        EditorApplication.delayCall += Apply;
    }

    public void OnPreprocessBuild(BuildReport report)
    {
        Apply();
    }

    static void Apply()
    {
        foreach (var importer in PluginImporter.GetAllImporters())
            if (importer.assetPath.StartsWith(NativePluginRoot, StringComparison.Ordinal))
                importer.SetIncludeInBuildDelegate(ExcludeFromPlayer);
    }

    static bool ExcludeFromPlayer(string path) => false;
}
