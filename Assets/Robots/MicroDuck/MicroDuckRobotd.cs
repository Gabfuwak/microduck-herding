using System;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Process = System.Diagnostics.Process;
using Stopwatch = System.Diagnostics.Stopwatch;

// Transport and process ownership only. All policy decisions remain in Pollen's robotd.
public sealed class MicroDuckRobotd : IDisposable
{
    public sealed class Motors
    {
        public double[] targets = new double[15];
        public int kp = 200;
        public bool torque, released;
    }
    readonly object gate = new object();
    readonly bool lockstep, captureTrace;
    bool tickRequested, tickCommitted;
    long tickTimeUs;
    long commandVersion, sentCommandVersion;
    volatile bool commandReady;
    StreamWriter commandWriter;
    StreamReader commandReader;
    int rpcId = 10;
    readonly TcpListener listener;
    readonly Process process;
    readonly Thread bodyThread, commandThread, stateThread;
    readonly string socketPath;
    readonly StreamWriter log;
    readonly object logGate = new object();
    volatile bool stopping;
    volatile string error, policy = "starting";
    volatile bool fallen;
    Motors motors = new Motors();
    string sensors;
    double[] command = new double[3];
    Socket commands, states;
    TcpClient bodyClient;
    public string Policy => policy;
    public bool Fallen => fallen;
    public string RunDirectory { get; }
    public long Reads { get; private set; }
    public long Writes { get; private set; }
    public Motors Output { get { lock (gate) return motors; } }

    public MicroDuckRobotd(string project, string initialSensors, bool lockstep = true, bool captureTrace = false)
    {
        this.lockstep = lockstep;
        this.captureTrace = captureTrace;
        sensors = initialSensors;
        string root = Environment.GetEnvironmentVariable("MICRODUCK_ROBOTD_ROOT");
        if (string.IsNullOrEmpty(root))
        {
            var directory = new DirectoryInfo(project);
            while (directory != null && !File.Exists(Path.Combine(directory.FullName, "Builds/Robotd/bin/robotd"))) directory = directory.Parent;
            root = directory != null ? Path.Combine(directory.FullName, "Builds/Robotd") : Path.Combine(project, "Builds/Robotd");
        }
        string binary = Path.Combine(root, lockstep ? "bin/robotd" : "bin/robotd-upstream");
        string ort = Path.Combine(root, "onnxruntime-linux-x64-1.23.2/lib/libonnxruntime.so.1.23.2");
        if (!File.Exists(binary) || !File.Exists(ort))
            throw new FileNotFoundException("Run python3 Tools/Robotd/prepare.py before using robotd.");
        string id = Guid.NewGuid().ToString("N").Substring(0, 12);
        socketPath = "/tmp/microduck-" + id + ".sock";
        RunDirectory = Path.Combine(root, "runs", DateTime.UtcNow.ToString("yyyyMMdd-HHmmss-") + id);
        Directory.CreateDirectory(RunDirectory);
        log = new StreamWriter(Path.Combine(RunDirectory, "robotd.log")) { AutoFlush = true };
        string config = Path.Combine(RunDirectory, "robotd.toml");
        string pol = Path.Combine(root, "policies");
        File.WriteAllText(config, "[policy]\nwalk = " + Quote(Path.Combine(pol, "velstand.onnx")) +
            "\nstand = \"none\"\nsitstand = " + Quote(Path.Combine(pol, "alpha_sitstand.onnx")) +
            "\nground_pick = " + Quote(Path.Combine(pol, "alpha_ground_pick.onnx")) +
            "\nkick_left = " + Quote(Path.Combine(pol, "ball_kick_left.onnx")) +
            "\nkick_right = " + Quote(Path.Combine(pol, "ball_kick_right.onnx")) +
            "\nroulade = " + Quote(Path.Combine(pol, "roulade.onnx")) +
            "\n[pickup]\nenabled = false\nmodel = " + Quote(Path.Combine(root, "models/pickup_detector.onnx")) +
            "\n[audio]\nenabled = false\n[idle]\nenabled = false\n");
        listener = new TcpListener(IPAddress.Loopback, 0);
        listener.Start();
        int port = ((IPEndPoint)listener.LocalEndpoint).Port;
        process = new Process();
        process.StartInfo.FileName = binary;
        process.StartInfo.Arguments = (lockstep ? "--sim-lockstep " : "") + "--sim 127.0.0.1:" + port + " --socket " + socketPath + " --params " + Quote(config);
        process.StartInfo.UseShellExecute = false;
        process.StartInfo.RedirectStandardOutput = process.StartInfo.RedirectStandardError = true;
        process.StartInfo.EnvironmentVariables["ORT_DYLIB_PATH"] = ort;
        process.StartInfo.EnvironmentVariables["DUCK_RUNTIME_DIR"] = RunDirectory;
        process.StartInfo.EnvironmentVariables["RUST_LOG"] = "info";
        process.OutputDataReceived += (_, e) => Log(e.Data);
        process.ErrorDataReceived += (_, e) => Log(e.Data);
        try
        {
            process.Start(); process.BeginOutputReadLine(); process.BeginErrorReadLine();
            bodyThread = Start(ServeBody, "MicroDuck body");
            commandThread = Start(Drive, "MicroDuck commands");
            stateThread = Start(Observe, "MicroDuck state");
        }
        catch { Dispose(); throw; }
    }

    static string Quote(string text) => JsonConvert.SerializeObject(text);
    Thread Start(Action action, string name)
    {
        var thread = new Thread(() => {
            try { action(); }
            catch (Exception e) { if (!stopping) { error = e.ToString(); Log(error); } }
        }) { IsBackground = true, Name = name };
        thread.Start(); return thread;
    }
    void Log(string text) { if (text != null) lock (logGate) { if (!stopping) log.WriteLine(text); } }
    public void Check()
    {
        if (error != null) throw new IOException("robotd bridge: " + error);
        if (process.HasExited) throw new IOException("robotd exited (" + process.ExitCode + "). See " + RunDirectory);
    }
    public void PublishSensors(string json) { lock (gate) sensors = json; }
    public void Command(double x, double y, double z)
    {
        lock (gate) { command = new[] { x, y, z }; commandVersion++; }
    }
    // Called only on Unity's physics thread, once per four MuJoCo substeps.
    // Socket workers never access native physics memory.
    public bool Step(string sensorJson, double simulationTime)
    {
        Check();
        if (!commandReady) return false;
        SendCommand();
        lock (gate)
        {
            sensors = sensorJson;
            tickTimeUs = (long)Math.Round(simulationTime * 1000000);
            tickCommitted = false;
            tickRequested = true;
            Monitor.PulseAll(gate);
            long started = Stopwatch.GetTimestamp();
            while (!tickCommitted)
            {
                Monitor.Wait(gate, 50);
                Check();
                if ((Stopwatch.GetTimestamp() - started) / (double)Stopwatch.Frequency > 2)
                    throw new TimeoutException("robotd did not finish its control tick; physical state retained.");
            }
        }
        return true;
    }
    void SendCommand()
    {
        double[] c;
        lock (gate)
        {
            if (sentCommandVersion == commandVersion) return;
            c = command; sentCommandVersion = commandVersion;
        }
        commandWriter.WriteLine(JsonConvert.SerializeObject(new { jsonrpc = "2.0", id = ++rpcId,
            method = "robot.move", @params = new { vx = c[0], vy = c[1], vyaw = c[2] } }));
        var response = JObject.Parse(commandReader.ReadLine() ?? throw new IOException("robotd closed command IPC."));
        if (response["error"] != null || (bool?)response["result"]?["accepted"] == false)
            throw new IOException("Movement refused: " + response);
    }
    void ServeBody()
    {
        while (!stopping)
        {
            using (var client = listener.AcceptTcpClient())
            {
                bodyClient = client; client.NoDelay = true;
                using var stream = client.GetStream();
                using var reader = new StreamReader(stream, Encoding.UTF8, false, 4096, true);
                using var writer = new StreamWriter(stream, new UTF8Encoding(false), 4096, true) { AutoFlush = true };
                bool hello = false;
                string line;
                while (!stopping && (line = reader.ReadLine()) != null)
                {
                    var req = JObject.Parse(line);
                    string op = (string)req["op"];
                    string reply = "{}";
                    lock (gate)
                    {
                        if (op == "hello")
                        {
                            hello = (int?)req["protocol"] == (lockstep ? 2 : 1) && (int?)req["joints"] == 15;
                            reply = hello ? JsonConvert.SerializeObject(new { protocol = lockstep ? 2 : 1 }) : "{\"error\":\"Wrong body protocol or joint count\"}";
                        }
                        else if (!hello) throw new IOException("Body request before valid handshake.");
                        else if (op == "tick" && lockstep)
                        {
                            if ((long?)req["period_us"] != 20000) throw new IOException("Expected 50 Hz robotd control.");
                            if (!tickRequested) Monitor.Wait(gate, 40);
                            bool advance = tickRequested;
                            tickRequested = false;
                            reply = JsonConvert.SerializeObject(new { time_us = tickTimeUs, advance });
                        }
                        else if (op == "commit" && lockstep)
                        {
                            tickCommitted = true; Monitor.PulseAll(gate);
                        }
                        else if (op == "read") { reply = sensors; Reads++; }
                        else if (op == "write")
                        {
                            var target = req["targets"].ToObject<double[]>();
                            if (target.Length != 15 || Array.Exists(target, x => double.IsNaN(x) || double.IsInfinity(x)))
                                throw new IOException("Invalid robotd motor targets.");
                            motors = new Motors { targets = target, kp = motors.kp, torque = motors.torque, released = motors.released };
                            Writes++;
                        }
                        else if (op == "gain") motors = new Motors { targets = motors.targets, kp = (int)req["kp"], torque = motors.torque, released = motors.released };
                        else if (op == "torque")
                        {
                            bool on = (bool)req["on"];
                            var target = on ? JObject.Parse(sensors)["positions"].ToObject<double[]>() : motors.targets;
                            motors = new Motors { targets = target, kp = motors.kp, torque = on, released = motors.released || on };
                        }
                        else if (op == "slow") reply = JsonConvert.SerializeObject(new { volts = 7.4, temps_c = new double[] {32,32,32,32,32,32,32,32,32,32,32,32,32,32,32} });
                        else throw new IOException("Unknown body operation: " + op);
                    }
                    writer.WriteLine(reply);
                }
            }
            bodyClient = null;
        }
    }
    Socket Connect()
    {
        var end = new UnixDomainSocketEndPoint(socketPath);
        for (int attempt = 0; !stopping && attempt < 500; attempt++)
        {
            var socket = new Socket(AddressFamily.Unix, SocketType.Stream, ProtocolType.Unspecified);
            try { socket.Connect(end); return socket; }
            catch (SocketException) { socket.Dispose(); Thread.Sleep(20); }
        }
        throw new IOException("robotd IPC did not become available.");
    }
    void Drive()
    {
        using var socket = Connect(); commands = socket;
        using var stream = new NetworkStream(socket, false);
        using var writer = new StreamWriter(stream, new UTF8Encoding(false)) { AutoFlush = true };
        using var reader = new StreamReader(stream);
        socket.ReceiveTimeout = socket.SendTimeout = 2000;
        commandWriter = writer; commandReader = reader;
        writer.WriteLine("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"robot.enable\",\"params\":{\"on\":true}}");
        var response = JObject.Parse(reader.ReadLine() ?? throw new IOException("No enable response."));
        if ((bool?)response["result"]?["accepted"] != true) throw new IOException("Enable refused: " + response);
        commandReady = true;
        while (!stopping)
        {
            if (!lockstep) SendCommand();
            Thread.Sleep(10);
        }
    }

    void Observe()
    {
        using var socket = Connect(); states = socket;
        using var stream = new NetworkStream(socket, false);
        using var writer = new StreamWriter(stream, new UTF8Encoding(false)) { AutoFlush = true };
        using var reader = new StreamReader(stream);
        writer.WriteLine("{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"robot.subscribe\",\"params\":{\"hz\":50}}");
        string line;
        using var trace = captureTrace ? new StreamWriter(Path.Combine(RunDirectory, "states.jsonl")) { AutoFlush = true } : null;
        while (!stopping && (line = reader.ReadLine()) != null)
        {
            var frame = JObject.Parse(line);
            if ((string)frame["method"] != "robot.state") continue;
            var state = frame["params"];
            policy = (string)state["policy"] ?? "unknown";
            fallen = (bool?)state["safety"]?["fallen"] ?? false;
            trace?.WriteLine(line);
        }
    }
    public void Dispose()
    {
        if (stopping) return;
        stopping = true;
        listener?.Stop();
        bodyClient?.Close(); commands?.Close(); states?.Close();
        if (process != null) { try { if (!process.HasExited) { process.Kill(); process.WaitForExit(2000); } } catch (InvalidOperationException) { } process.Dispose(); }
        bodyThread?.Join(500); commandThread?.Join(500); stateThread?.Join(500);
        lock (logGate) log?.Dispose();
        try { File.Delete(socketPath); File.Delete(socketPath + ".lock"); } catch (IOException) { }
    }
}
