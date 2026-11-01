using System;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text;
using UnityEngine;

// A data-only binding to Pollen's controller. The native library owns policy
// observations, action history, filtering and recovery; Unity owns MuJoCo.
internal sealed unsafe class MicroDuckPollenControl : IDisposable
{
    const string Library = "microduck_control";
    IntPtr handle;

    [StructLayout(LayoutKind.Sequential)]
    internal struct SensorSnapshot
    {
        public fixed double positions[15];
        public fixed double velocities[15];
        public fixed double currents_ma[15];
        public fixed double gyro[3];
        public fixed double gravity[3];
        public fixed double quat[4]; // MuJoCo/Pollen w,x,y,z
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct VelocityCommand
    {
        public double forward, left, yaw_rate;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct MotorOutput
    {
        public fixed double targets[15];
        public uint gain, torque, state, fallen;
    }

    [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
    static extern uint md_abi_version();
    [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
    static extern int md_create([MarshalAs(UnmanagedType.LPUTF8Str)] string models,
        [MarshalAs(UnmanagedType.LPUTF8Str)] string ort, out IntPtr result);
    [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
    static extern int md_step(IntPtr control, ref SensorSnapshot sensors,
        ref VelocityCommand command, ulong simulationTimeUs, out MotorOutput result);
    [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
    static extern int md_reset(IntPtr control);
    [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
    static extern void md_destroy(IntPtr control);
    [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
    static extern IntPtr md_last_error();

    internal MicroDuckPollenControl(TextAsset[] policies)
    {
        if (sizeof(SensorSnapshot) != 440 || sizeof(VelocityCommand) != 24 || sizeof(MotorOutput) != 136)
            throw new InvalidOperationException("Pollen control struct layout does not match ABI 1.");
        if (md_abi_version() != 1)
            throw new InvalidOperationException("Packaged Pollen controller ABI must be version 1.");
        Check(md_create(StagePolicies(policies), RuntimePath(), out handle));
        if (handle == IntPtr.Zero)
            throw new InvalidOperationException("Pollen controller returned an empty handle.");
    }

    internal void Step(ref SensorSnapshot sensors, ref VelocityCommand command,
        ulong simulationTimeUs, out MotorOutput result)
    {
        if (handle == IntPtr.Zero) throw new ObjectDisposedException(nameof(MicroDuckPollenControl));
        Check(md_step(handle, ref sensors, ref command, simulationTimeUs, out result));
    }

    // Only explicit placement may restart the controller clock and history.
    internal void ResetPlacement()
    {
        if (handle == IntPtr.Zero) throw new ObjectDisposedException(nameof(MicroDuckPollenControl));
        Check(md_reset(handle));
    }

    public void Dispose()
    {
        if (handle == IntPtr.Zero) return;
        md_destroy(handle);
        handle = IntPtr.Zero;
    }

    static void Check(int status)
    {
        if (status != 0)
            throw new InvalidOperationException("Pollen control: " + Marshal.PtrToStringAnsi(md_last_error()));
    }

    static string RuntimePath()
    {
#if UNITY_ANDROID && !UNITY_EDITOR
        // Unity packages this arm64 library beside the control plugin. Android's
        // native loader resolves its name without extracting APK assets ourselves.
        return "libonnxruntime.so";
#elif UNITY_EDITOR_LINUX || UNITY_STANDALONE_LINUX
        var candidates = new[] {
            Path.Combine(Application.dataPath, "Plugins/Linux/x86_64/libonnxruntime.so"),
            Path.Combine(Application.dataPath, "Plugins/x86_64/libonnxruntime.so"),
            Path.Combine(Application.dataPath, "Plugins/libonnxruntime.so")
        };
        return candidates.FirstOrDefault(File.Exists) ??
            throw new FileNotFoundException("The packaged Linux ONNX Runtime library is missing.");
#else
        throw new PlatformNotSupportedException("MicroDuck currently packages Linux x86-64 and Quest Android ARM64.");
#endif
    }

    static string StagePolicies(TextAsset[] assets)
    {
        if (assets == null || assets.Length == 0 || assets.Any(asset => asset == null))
            throw new InvalidOperationException("Assign the packaged Pollen policy byte assets.");
        var ordered = assets.OrderBy(asset => asset.name, StringComparer.Ordinal).ToArray();
        if (ordered.Select(asset => asset.name).Distinct(StringComparer.Ordinal).Count() != ordered.Length)
            throw new InvalidOperationException("Pollen policy filenames must be unique.");
        var bytes = ordered.Select(asset => asset.bytes).ToArray();
        using var sha = SHA256.Create();
        var fingerprints = new StringBuilder();
        for (var i = 0; i < ordered.Length; i++)
        {
            var name = ordered[i].name;
            if (Path.GetFileName(name) != name || bytes[i].Length == 0)
                throw new InvalidOperationException("Invalid packaged Pollen asset: " + name);
            fingerprints.Append(name).Append(':').Append(BitConverter.ToString(sha.ComputeHash(bytes[i]))).Append('\n');
        }
        var hash = BitConverter.ToString(sha.ComputeHash(Encoding.UTF8.GetBytes(fingerprints.ToString())))
            .Replace("-", "").ToLowerInvariant();
        var directory = Path.Combine(Application.persistentDataPath, "Pollen", hash);
        Directory.CreateDirectory(directory);
        for (var i = 0; i < ordered.Length; i++)
        {
            var path = Path.Combine(directory, ordered[i].name);
            if (File.Exists(path) && sha.ComputeHash(File.ReadAllBytes(path)).SequenceEqual(sha.ComputeHash(bytes[i])))
                continue;
            // Pollen reads files directly. TextAssets work equally in the Editor
            // and Android APK, so a clone needs no external model downloads.
            var temporary = path + ".tmp";
            File.WriteAllBytes(temporary, bytes[i]);
            if (File.Exists(path)) File.Delete(path);
            File.Move(temporary, path);
        }
        return directory;
    }
}
