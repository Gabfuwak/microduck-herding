using System;
using System.Diagnostics;
using UnityEngine;

internal sealed class MicroDuckPerformance
{
    internal enum Timing
    {
        Frame,
        SurfaceScan,
        SceneXml,
        Vfs,
        ModelCompile,
        Policy,
        SurfacePose
    }

    const int TimingSampleCapacity = 1024;
    const int FrameSampleCapacity = 4096;
    static readonly double MillisecondsPerTick = 1000.0 / Stopwatch.Frequency;

    sealed class SampleWindow
    {
        readonly double[] samples;
        int next;
        int count;

        internal SampleWindow(int capacity)
        {
            samples = new double[capacity];
        }

        internal void Add(double milliseconds)
        {
            samples[next] = milliseconds;
            next++;
            if (next == samples.Length)
                next = 0;
            if (count < samples.Length)
                count++;
        }

        internal double[] GetSortedCopy()
        {
            var copy = new double[count];
            int start = count == samples.Length ? next : 0;
            for (int i = 0; i < count; i++)
                copy[i] = samples[(start + i) % samples.Length];
            Array.Sort(copy);
            return copy;
        }
    }

    [Serializable]
    sealed class TimingReport
    {
        public string timing;
        public long totalCount;
        public double totalMs;
        public double maxMs;
        public int windowSize;
        public double windowP50Ms;
        public double windowP95Ms;
        public double windowP99Ms;
    }

    [Serializable]
    sealed class Report
    {
        public string frameDurationMeaning = "wall-clock elapsed time; not GPU/CPU attribution";
        public string lifetimeMetricsMeaning = "totalCount, totalMs, and maxMs cover the capture lifetime";
        public string rollingMetricsMeaning = "window percentiles cover the most recent samples, up to the fixed window capacity";
        public TimingReport[] timings;
        public long geometryRebuilds;
        public long poseRefreshes;
        public long posesUpdated;
        public long unchangedRefreshes;
        public long geometryChangeRebuilds;
        public long surfaceAdditionRebuilds;
        public long surfaceRemovalRebuilds;
    }

    readonly SampleWindow[] windows;
    readonly long[] totalCounts;
    readonly double[] totalMilliseconds;
    readonly double[] maximumMilliseconds;
    long geometryRebuilds;
    long poseRefreshes;
    long posesUpdated;
    long unchangedRefreshes;
    long geometryChangeRebuilds;
    long surfaceAdditionRebuilds;
    long surfaceRemovalRebuilds;

    internal MicroDuckPerformance()
    {
        int timingCount = Enum.GetValues(typeof(Timing)).Length;
        windows = new SampleWindow[timingCount];
        totalCounts = new long[timingCount];
        totalMilliseconds = new double[timingCount];
        maximumMilliseconds = new double[timingCount];
        for (int i = 0; i < timingCount; i++)
            windows[i] = new SampleWindow(i == (int)Timing.Frame ? FrameSampleCapacity : TimingSampleCapacity);
    }

    internal void Record(Timing timing, long startedAt)
    {
        int index = (int)timing;
        if ((uint)index >= (uint)windows.Length)
            return;

        long elapsedTicks = Stopwatch.GetTimestamp() - startedAt;
        if (elapsedTicks < 0)
            return;
        AddSample(index, elapsedTicks * MillisecondsPerTick);
    }

    internal void RecordFrame(float seconds)
    {
        if (float.IsNaN(seconds) || float.IsInfinity(seconds) || seconds < 0f)
            return;
        AddSample((int)Timing.Frame, seconds * 1000.0);
    }

    internal void RecordSurfaceRefresh(bool rebuilt, int updatedPoses)
    {
        if (rebuilt)
        {
            geometryRebuilds++;
            return;
        }

        if (updatedPoses > 0)
        {
            poseRefreshes++;
            posesUpdated += updatedPoses;
        }
        else
        {
            unchangedRefreshes++;
        }
    }

    internal void RecordRebuildReasons(bool geometryChanged, bool added, bool removed)
    {
        if (geometryChanged) geometryChangeRebuilds++;
        if (added) surfaceAdditionRebuilds++;
        if (removed) surfaceRemovalRebuilds++;
    }

    internal string ToJson()
    {
        var timingReports = new TimingReport[windows.Length];
        for (int i = 0; i < windows.Length; i++)
        {
            double[] sorted = windows[i].GetSortedCopy();
            timingReports[i] = new TimingReport
            {
                timing = ((Timing)i).ToString(),
                totalCount = totalCounts[i],
                totalMs = totalMilliseconds[i],
                maxMs = maximumMilliseconds[i],
                windowSize = sorted.Length,
                windowP50Ms = Percentile(sorted, 0.50),
                windowP95Ms = Percentile(sorted, 0.95),
                windowP99Ms = Percentile(sorted, 0.99)
            };
        }

        return JsonUtility.ToJson(new Report
        {
            timings = timingReports,
            geometryRebuilds = geometryRebuilds,
            poseRefreshes = poseRefreshes,
            posesUpdated = posesUpdated,
            unchangedRefreshes = unchangedRefreshes,
            geometryChangeRebuilds = geometryChangeRebuilds,
            surfaceAdditionRebuilds = surfaceAdditionRebuilds,
            surfaceRemovalRebuilds = surfaceRemovalRebuilds
        });
    }

    void AddSample(int index, double milliseconds)
    {
        windows[index].Add(milliseconds);
        totalCounts[index]++;
        totalMilliseconds[index] += milliseconds;
        if (totalCounts[index] == 1 || milliseconds > maximumMilliseconds[index])
            maximumMilliseconds[index] = milliseconds;
    }

    static double Percentile(double[] sorted, double percentile)
    {
        if (sorted.Length == 0)
            return 0.0;

        // Nearest-rank percentile: rank = ceil(p * n), with one-based rank.
        int index = (int)Math.Ceiling(percentile * sorted.Length) - 1;
        if (index < 0)
            index = 0;
        return sorted[index];
    }
}
