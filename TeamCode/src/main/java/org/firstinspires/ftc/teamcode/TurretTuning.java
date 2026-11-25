package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;


/**
 * Turret aiming settings: THURSDAY's values (with Thursday's aim code, TurretAimer, and Thursday's
 * 4.0 ticks per degree in mechanumtest). You can change them live in Panels (Configurables >
 * TurretTuning) while the main TeleOp runs; Panels changes are not saved, so write the numbers you
 * like in here. (Friday's softer belt values, if it ever wobbles: aimKP 10, aimKI 2, deadband 1,
 * max speed 250, accel 2000, motor P 10, I 1.)
 *
 * If the turret wobbles or hunts back and forth on its belt:
 *   - FIRST check the ticks per degree (shown on the Driver Station during INIT): measure it with the
 *     "Ticks Per Degree" OpMode. A wrong ticks-per-degree makes every turret move too big (looks like wobble).
 *   - then lower aimKP (snappiness) and aimKI (catch-up),
 *   - then lower motorP / motorI (the motor's own speed loop).
 * If it is too slow to follow the target, raise them back up a step at a time.
 */
@Configurable
public class TurretTuning {
    /** Turret speed (deg/s) per degree of aim error. Thursday: 18. */
    public static double aimKP = 18.0;
    /** Extra speed that builds up while the turret keeps trailing the goal. Thursday: 5. */
    public static double aimKI = 5.0;
    /** Inside this many degrees the turret stops correcting (stops buzzing on the target). Thursday: 0.5. */
    public static double deadbandDeg = 0.5;
    /** Fastest the turret may spin while aiming, deg/s (also capped by what the motor can do). Thursday: 700. */
    public static double maxAimSpeedDegPerSec = 700.0;
    /** How fast the turret may SPEED UP, deg/s per s. 0 = no limit (Thursday). Friday's belt value: 2000. */
    public static double maxAccelDegPerSec2 = 0.0;
    /** Fastest way: in the blind spot, wait on the side where the turret will point at the goal again
     *  soonest (from how the robot is turning). false = Thursday's nearest-side rule. */
    public static boolean fastestWay = true;

    /** The hub's own velocity loop for the turret motor. Thursday: P 15, I 3, D 0, F 12.5. */
    public static double motorP = 15.0;
    public static double motorI = 3.0;
    public static double motorD = 0.0;
    public static double motorF = 12.5;

    /**
     * Testing without the field: set this to the ID of any AprilTag (e.g. one on a scoreboard; the
     * Driver Station TAGS line shows the IDs the camera sees). The turret then aims at that tag as if
     * it were your alliance's first hive. -1 = off.
     */
    public static int testTagId = -1;
}
