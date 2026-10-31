using System;
using Phase = MicroDuckFallRecovery.Phase;

static class Program
{
    const double Dt = 0.02;
    static int passed;
    static readonly double[] Positions = new double[14];
    static bool Tick(MicroDuckFallRecovery r, double tilt, double pitchRate) =>
        r.Step(Math.Sin(tilt), 0, -Math.Cos(tilt), 0, pitchRate, 0, Positions, Dt);
    static void Check(bool condition, string message) { if (!condition) throw new Exception(message); }
    static MicroDuckFallRecovery Armed()
    {
        var r = new MicroDuckFallRecovery();
        for (int i = 0; i < 10; i++) Tick(r, 0, 0);
        return r;
    }
    static MicroDuckFallRecovery Falling()
    {
        var r = Armed();
        Tick(r, 0.7, 3); Tick(r, 0.7, 3);
        Check(r.Current == Phase.Policy, "No limp before 60 ms debounce");
        Tick(r, 0.7, 3);
        Check(r.Current == Phase.Limp, "A predicted fall must soften the motors");
        return r;
    }
    static void Test(string name, Action test) { test(); passed++; Console.WriteLine("PASS " + name); }
    static void Main()
    {
        Test("analytic projected-gravity derivative", () => Check(Math.Abs(MicroDuckFallRecovery.GravityZRate(0.5, 0, 0, 1) - 0.5) < 1e-12, "frame/sign mismatch"));
        Test("upright footfall cannot cause a fall", () => { var r = Armed(); for (int i=0;i<100;i++) Tick(r, 0.14, 3); Check(r.Current == Phase.Policy, "false positive"); });
        Test("recovering lean cannot cause a fall", () => { var r = Armed(); for (int i=0;i<100;i++) Tick(r, 0.9, -4); Check(r.Current == Phase.Policy, "recovering is not falling"); });
        Test("already-down robot stays policy-driven", () => { var r = new MicroDuckFallRecovery(); for (int i=0;i<100;i++) Tick(r, 1.8, 0); Check(r.Current == Phase.Policy, "fallen verdict must not gate the policy"); });
        Test("predictive fall is debounced", () => Falling());
        Test("fast falls can fire before fallen-report debounce", () => { var r = Armed(); for(int i=0;i<3;i++) Tick(r,1.4,3); Check(r.Current==Phase.Limp,"instant fallen classification must not suppress the early predictor"); });
        Test("limp follows measured joints", () => { var r = Falling(); Check(r.Target(0, 0.37, 0) == 0.37, "a fixed hold target fights the landing"); });
        Test("stillness is debounced", () => { var r = Falling(); for(int i=0;i<9;i++) Tick(r,1.8,0); Check(r.Current == Phase.Limp,"one quiet moment is not landing"); Tick(r,1.8,0); Check(r.Current == Phase.Posing,"200ms landing"); });
        Test("limp has a finite timeout", () => { var r = Falling(); for(int i=0;i<75;i++) Tick(r,1.8,2); Check(r.Current == Phase.Posing,"must not remain limp indefinitely"); });
        Test("pose ramp begins from real landed joints", () => { var r=Falling(); Positions[0]=0.8; for(int i=0;i<10;i++) Tick(r,1.8,0); Check(Math.Abs(r.Target(0,9,0)-0.8)<1e-12,"must not jump at ramp start"); for(int i=0;i<15;i++) Tick(r,1.8,0); Check(Math.Abs(r.Target(0,9,0)-0.4)<1e-12,"linear halfway target"); Check(Positions[0]==0.8,"must not overwrite physical state"); Positions[0]=0; });
        Test("handback does not wait for upright", () => { var r=Falling(); for(int i=0;i<10;i++) Tick(r,1.8,0); bool handedBack=false; for(int i=0;i<30;i++) handedBack|=Tick(r,1.8,0); Check(handedBack && r.Current==Phase.Policy,"old deadlock regression: body still down, actor must run"); });
        Test("no repeated limp while getting up", () => { var r=Falling(); for(int i=0;i<40;i++) Tick(r,1.8,0); for(int i=0;i<200;i++) Tick(r,1.0,2); Check(r.Current==Phase.Policy,"predictor cannot rearm from a prone recovery"); });
        Test("standing rearms a later independent fall", () => { var r=Falling(); for(int i=0;i<40;i++) Tick(r,1.8,0); for(int i=0;i<10;i++) Tick(r,0,0); for(int i=0;i<3;i++) Tick(r,0.7,3); Check(r.Current==Phase.Limp,"must catch subsequent falls"); });
        Console.WriteLine($"{passed} recovery regression cases passed.");
    }
}
