using System;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;

/// <summary>
/// Assimp is used only to import robot meshes in the editor. Players use the saved
/// Unity meshes, not runtime URDF import. The URDF package's desktop native plug-ins
/// incorrectly enable Any Platform, including Android, and must not ship in players.
/// Build filtering preserves editor import support without editing PackageCache.
/// </summary>
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
