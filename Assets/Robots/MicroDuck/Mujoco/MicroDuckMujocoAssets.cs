using System;
using System.Collections;
using System.IO;
using UnityEngine;
using UnityEngine.Networking;

// Android StreamingAssets lives inside the APK. MuJoCo's mesh loader needs real
// files, so copy this model into the app's private storage before initialization.
public static class MicroDuckMujocoAssets
{
    public static IEnumerator Prepare(Action<string> ready, Action<Exception> failed)
    {
#if UNITY_ANDROID && !UNITY_EDITOR
        string directory = Path.Combine(Application.persistentDataPath, "MicroDuck-3.15.0");
        var files = new[] {
            "robot.xml", "assets/bottom_head_shell.stl", "assets/hip_l.stl",
            "assets/jaw.stl", "assets/leg.stl", "assets/np_f970.stl",
            "assets/power_support.stl", "assets/sole_left.stl",
            "assets/sole_right.stl", "assets/top_head_shell.stl"
        };
        // Recopy on launch so a newly installed APK cannot use stale model files.
        foreach (string file in files)
        {
            string url = Application.streamingAssetsPath + "/MicroDuck/" + file;
            using (var request = UnityWebRequest.Get(url))
            {
                yield return request.SendWebRequest();
                if (request.result != UnityWebRequest.Result.Success)
                {
                    failed(new IOException($"Cannot extract {file}: {request.error}"));
                    yield break;
                }
                Exception writeError = null;
                try
                {
                    string destination = Path.Combine(directory, file);
                    Directory.CreateDirectory(Path.GetDirectoryName(destination));
                    File.WriteAllBytes(destination, request.downloadHandler.data);
                }
                catch (Exception error) { writeError = error; }
                if (writeError != null) { failed(writeError); yield break; }
            }
        }
        Debug.Log("MicroDuck: Android MJCF and collision assets extracted.");
#else
        string directory = Path.Combine(Application.streamingAssetsPath, "MicroDuck");
#endif
        ready(directory);
        yield break;
    }
}
