using System.Linq;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEngine.XR.Management;
using UnityEngine.XR.OpenXR;

// The Linux trial has an ordinary camera. Keep its player free of XR startup
// while retaining AR simulation in the Editor and OpenXR in the Quest build.
sealed class MicroDuckDesktopBuildSettings : BuildPlayerProcessor, IPreprocessBuildWithReport
{
    static bool s_IsDesktopTrial;

    public override int callbackOrder => int.MaxValue;

    public override void PrepareForBuild(BuildPlayerContext context)
    {
        var options = context.BuildPlayerOptions;
        s_IsDesktopTrial = options.target == BuildTarget.StandaloneLinux64 &&
            options.scenes != null && options.scenes.Length == 1 &&
            options.scenes[0] == MicroDuckNavigationTrial.ScenePath;
    }

    public void OnPreprocessBuild(BuildReport report)
    {
        if (!s_IsDesktopTrial || report.summary.platform != BuildTarget.StandaloneLinux64) return;
        s_IsDesktopTrial = false;

        // Stock XR processors inject these even without an OpenXR loader. Remove
        // them after those callbacks so disabled Android feature assets are not
        // serialized into this player and XR Simulation does not start its camera.
        // Their stock postprocessors also remove these assets after every build.
        PlayerSettings.SetPreloadedAssets(PlayerSettings.GetPreloadedAssets()
            .Where(asset => asset is not XRGeneralSettings && asset is not OpenXRSettings).ToArray());
    }
}
