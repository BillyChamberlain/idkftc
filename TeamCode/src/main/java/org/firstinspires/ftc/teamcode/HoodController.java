package org.firstinspires.ftc.teamcode;

/**
 * HoodController moves a hood driven by continuous-rotation servos to a target
 * position, even though CR servos can't report where they are.
 *
 * How: at the start of TeleOp it drives the hood down into its bottom stop
 * (that's position 0). After that it keeps track of position by counting how
 * long, and how hard, the servos have been driven:
 *     position += power * seconds / fullTravelSec
 * Position 0 = bottom stop, 1 = top stop. It re-homes every time TeleOp
 * starts. If the hood drifts during a long practice, restart the OpMode.
 *
 * MEASURE fullTravelSec: from the bottom stop, run the hood at full power and time
 * how long it takes to reach the top stop.
 *
 * Plain Java with no hardware, so it can be tested on a laptop.
 */
public class HoodController {
    /** Seconds at full power from bottom stop to top stop. MEASURE THIS on the robot. */
    public double fullTravelSec = 1.2;
    /** Homing drives down for a full stroke times this, so it reaches the bottom stop from anywhere. */
    public double homeMargin = 1.3;
    /** Power per unit of position error. */
    public double kP = 5.0;
    /** CR servos barely move below this power, so never ask for less while correcting. */
    public double minPower = 0.12;
    /** Close enough. */
    public double tolerance = 0.015;
    /** Power sign that moves the hood UP (the original RT gave +1). */
    public int upSign = +1;

    private double position = 0;
    private boolean homed = false;
    private double homeStartMs = Double.NaN;

    /**
     * @param target wanted position 0..1
     * @return CR servo power to send to both hood servos
     */
    public double update(double nowMs, double dtSec, double target) {
        if (!homed) {
            if (Double.isNaN(homeStartMs)) homeStartMs = nowMs;
            if (nowMs - homeStartMs < fullTravelSec * homeMargin * 1000.0) return -upSign;
            homed = true;
            position = 0;
        }

        double up;  // + = up, in "full power" units
        target = Math.max(0, Math.min(1, target));
        double error = target - position;
        if (Math.abs(error) <= tolerance) {
            up = 0;
        } else {
            up = kP * error;
            if (Math.abs(up) < minPower) up = Math.signum(up) * minPower;
            up = Math.max(-1, Math.min(1, up));
        }

        if (dtSec > 0 && dtSec < 0.25) {
            position += up * dtSec / fullTravelSec;
            position = Math.max(0, Math.min(1, position));
        }
        return up * upSign;
    }

    public double position() { return position; }

    public boolean isHoming() { return !homed; }
}
