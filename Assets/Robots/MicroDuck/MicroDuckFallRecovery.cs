using System;

// Port of Pollen Robotics microduck b2b0d213: duck-control/src/fall.rs and
// robotd/src/main.rs (LimpFall/Landing). Times are simulated seconds at 50 Hz.
// This class emits motor targets only; it cannot alter a simulator's body state.
public sealed class MicroDuckFallRecovery
{
    public enum Phase { Policy, Limp, Posing }
    public Phase Current { get; private set; }
    public double Elapsed { get; private set; }
    readonly double[] landedPose = new double[14];
    double uprightFor, fallingFor, stillFor, downFor;
    public bool Fallen => downFor + Epsilon >= 0.2;
    bool armed;
    const double Epsilon = 1e-9;

    public void Initialize()
    {
        Current = Phase.Policy;
        Elapsed = stillFor = downFor = 0;
        ResetPredictor();
    }

    void ResetPredictor() { uprightFor = fallingFor = 0; armed = false; }

    public static double GravityZRate(double gx, double gy, double wx, double wy) => -(wx * gy - wy * gx);

    // True only on hand-back. The caller restarts policy history, as robotd does,
    // while preserving positions, velocities, contacts and the simulation clock.
    public bool Step(double gx, double gy, double gz, double wx, double wy, double wz,
        double[] positions, double dt)
    {
        // Pollen safety.rs: a debounced report, not an actor-enable condition.
        downFor = gz > -0.5 ? downFor + dt : 0;
        if (Current == Phase.Limp)
        {
            Elapsed += dt;
            stillFor = wx * wx + wy * wy + wz * wz < 1.0 ? stillFor + dt : 0;
            if (stillFor + Epsilon >= 0.2 || Elapsed + Epsilon >= 1.5)
            {
                Array.Copy(positions, landedPose, landedPose.Length);
                Current = Phase.Posing;
                Elapsed = 0;
            }
            return false;
        }
        if (Current == Phase.Posing)
        {
            Elapsed += dt;
            if (Elapsed + Epsilon < 0.6) return false;
            Current = Phase.Policy;
            ResetPredictor();
            return true;
        }
        if (Fallen) { ResetPredictor(); return false; }
        if (gz <= -0.90)
        {
            uprightFor += dt;
            if (uprightFor + Epsilon >= 0.2) armed = true;
        }
        else uprightFor = 0;
        double rate = GravityZRate(gx, gy, wx, wy);
        // Pollen's fallen verdict gates nothing. Its predictor only catches a
        // new fall from upright, not a robot already down or getting back up.
        bool goingDown = gz > -0.90 && rate > 0 && gz + rate * 0.3 > -0.5;
        if (!goingDown || !armed) { fallingFor = 0; return false; }
        fallingFor += dt;
        if (fallingFor + Epsilon >= 0.06)
        {
            ResetPredictor();
            Current = Phase.Limp;
            Elapsed = stillFor = 0;
        }
        return false;
    }

    public double Target(int joint, double measuredPosition, double home)
    {
        if (Current == Phase.Limp) return measuredPosition;
        if (Current != Phase.Posing) throw new InvalidOperationException("The policy owns targets now.");
        double t = Math.Min(1, Elapsed / 0.6);
        return landedPose[joint] + (home - landedPose[joint]) * t;
    }
}
