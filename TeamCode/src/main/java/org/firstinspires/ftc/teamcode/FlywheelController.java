package org.firstinspires.ftc.teamcode;

/**
 * FlywheelController holds the flywheel at a target speed so back-to-back
 * shots leave at the same speed.
 *
 *   power = feed-forward (what it takes to hold this speed at this battery voltage)
 *         + P (push harder the further below target we are)
 *         + I (slowly removes any small steady error)
 *   and when a ball pulls the speed down by more than boostBelowTps, full power
 *   until it recovers. That's the fastest possible recovery between shots.
 *
 * The motor runs in RUN_WITHOUT_ENCODER so this loop is in charge (the encoder
 * still counts, so getVelocity() works).
 *
 * Plain Java with no hardware, so it can be tested on a laptop.
 */
public class FlywheelController {
    /** Power per tick/s at nominalVolts. ~1/2800 for a 6000 rpm goBILDA motor (your old F of 11.7 matches). */
    public double kV = 1.0 / 2700.0;
    /** Power needed just to overcome friction. */
    public double kS = 0.03;
    /** Extra power per tick/s below target. */
    public double kP = 0.002;
    /** Power per (tick/s * second) of accumulated error. */
    public double kI = 0.0008;
    /** Biggest the I term can get (power). */
    public double maxI = 0.12;
    /** If speed is this far below target (a ball just went through), go full power. */
    public double boostBelowTps = 100.0;
    /** "Ready to shoot" when within this many ticks/s of the target. */
    public double readyToleranceTps = 40.0;
    public double nominalVolts = 12.0;

    private double integral = 0;     // tick/s * s
    private double lastTarget = 0;
    private boolean boosting = false;

    /**
     * @param dtSec   seconds since the last call
     * @return motor power 0..1 (never negative: we never brake the flywheel)
     */
    public double update(double targetTps, double measuredTps, double batteryVolts, double dtSec) {
        if (targetTps <= 0) {
            reset();
            return 0;
        }
        // A big target change makes the old I term meaningless
        if (Math.abs(targetTps - lastTarget) > 150) integral = 0;
        lastTarget = targetTps;

        double volts = batteryVolts > 7 ? batteryVolts : nominalVolts;
        double ff = (kS + kV * targetTps) * nominalVolts / volts;
        double error = targetTps - measuredTps;

        boosting = error > boostBelowTps;
        if (boosting) return 1.0;

        // Only build up I when we're close, so a spin-up doesn't overshoot later
        if (Math.abs(error) < boostBelowTps && dtSec > 0 && dtSec < 0.25) {
            integral += error * dtSec;
            double cap = maxI / Math.max(kI, 1e-9);
            integral = Math.max(-cap, Math.min(cap, integral));
        }
        double power = ff + kP * error + kI * integral;
        return Math.max(0, Math.min(1, power));
    }

    public boolean isReady(double targetTps, double measuredTps) {
        return targetTps > 0 && Math.abs(targetTps - measuredTps) <= readyToleranceTps;
    }

    public boolean isBoosting() { return boosting; }

    public void reset() {
        integral = 0;
        lastTarget = 0;
        boosting = false;
    }
}
