package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.io.FileWriter;
import java.util.Locale;

/**
 * TICKS PER DEGREE: measure how many turret-motor encoder ticks make ONE degree of turret turn.
 * EVERY OpMode uses the number saved here: the main TeleOp (auto-aim, turret limits, Sentry turret mode),
 * the Sentry OpMode and the Limits Tuner. (Until something is saved, they use Thursday's 4.0.)
 *
 * How to use it:
 *   1. Point the turret STRAIGHT FORWARD, then press INIT. That position is zero.
 *      (Forgot? After PLAY, line it up and press B: "this is forward".)
 *   2. Press PLAY. Turn the turret EXACTLY 90 degrees (a quarter turn) to the LEFT. Use the LEFT: the
 *      right side only has about 45 degrees of room before the wires.
 *        D-pad left/right   move it (slow)        left stick X   move it (fine)
 *        Y                  hand mode on/off: the motor lets go so you can turn the turret by hand
 *   3. Press A. It saves (ticks turned) / 90. Everything uses it from its next INIT.
 *   4. Hold X to drive the turret back to forward before you STOP (the next OpMode's INIT needs it
 *      pointing forward).
 *        BACK               delete it: everything goes back to Thursday's 4.0
 *
 * The D-pad and stick stop at the turret limits saved by the Limits Tuner (they're in ticks, so they're
 * right whatever the ticks per degree is). Hand mode can't be stopped, so the screen warns instead.
 * Tips: line the turret up with the robot's frame or a square to see 90 degrees exactly; a 3 degree
 * mistake makes the number about 3% off. The Driver Station shows the number live before you press A.
 * The file is the same one the Limits Tuner camera check (RB) writes; whichever was saved last is used.
 */
@TeleOp(name = "Ticks Per Degree", group = "Tuning")
public class TicksPerDegreeTuner extends OpMode {

    /** Written as the 2nd line of the file, so every OpMode knows this value may be used. */
    static final String METHOD = "90-degree tuner";

    private static final double DPAD_POWER = 0.25;
    private static final double STICK_MAX_POWER = 0.4;
    private static final int LIMIT_MARGIN_TICKS = 40;   // stop this far before a saved limit

    private DcMotorEx turret;
    private boolean handMode = false;
    private boolean prevA, prevB, prevY, prevBack;
    private boolean saved = false;
    private String message = "";
    private String inUse = "";        // "Now in use" text (files are read at INIT and after A/BACK, not every loop)
    private int[] savedLimits;        // the Limits Tuner's saved limits {min, max} in ticks from forward, or null

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "aim");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // zero = where the turret points now (forward)
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inUse = inUseText();
        savedLimits = LimitsTuner.loadLimits();
        telemetry.addLine("TICKS PER DEGREE: zero = where the turret points NOW. It must point STRAIGHT FORWARD.");
        telemetry.addLine("(Not forward? Press PLAY, line it up, then press B.)");
        telemetry.addLine("Then turn the turret exactly 90 degrees to the LEFT and press A.");
        telemetry.addLine("Now in use: " + inUse);
    }

    @Override
    public void loop() {
        int t = turret.getCurrentPosition();

        // ---- Y: hand mode (the motor goes limp so the turret can be turned by hand) ----
        if (gamepad1.y && !prevY) {
            handMode = !handMode;
            turret.setZeroPowerBehavior(handMode ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
            message = handMode ? "Hand mode ON: turn the turret by hand. Y = motor holds again." : "Hand mode OFF: the motor holds the turret.";
        }

        // ---- B: "the turret points forward now" (re-zero, if it wasn't forward at INIT) ----
        if (gamepad1.b && !prevB) {
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            t = 0;
            message = "Zero set here (forward). Now turn exactly 90 degrees LEFT and press A.";
        }

        // ---- Move the turret: D-pad (slow), left stick (fine), or hold X = back to forward. Not in hand mode. ----
        double power = 0;
        if (!handMode) {
            if (gamepad1.x) power = Math.abs(t) < 10 ? 0 : Math.max(-0.2, Math.min(0.2, -t * 0.002));  // back to zero, slowly
            else if (gamepad1.dpad_right) power = DPAD_POWER;
            else if (gamepad1.dpad_left) power = -DPAD_POWER;
            else if (Math.abs(gamepad1.left_stick_x) > 0.05) power = gamepad1.left_stick_x * STICK_MAX_POWER;
        }
        // Never drive past the Limits Tuner's saved limits (same zero: forward). Positive power = more ticks.
        boolean atMax = savedLimits != null && t >= savedLimits[1] - LIMIT_MARGIN_TICKS;
        boolean atMin = savedLimits != null && t <= savedLimits[0] + LIMIT_MARGIN_TICKS;
        if ((power > 0 && atMax) || (power < 0 && atMin)) power = 0;
        turret.setPower(power);

        // ---- What A would save right now ----
        double tpd = Math.abs(t) / 90.0;
        boolean usable = tpd >= 1 && tpd < 40;   // the same range every OpMode accepts when it loads the file
        // Which way the code thinks the turret turned (TURRET_DIR in mechanumtest: more ticks x TURRET_DIR = right)
        String way = t * mechanumtest.TURRET_DIR >= 0 ? "RIGHT" : "LEFT";

        // ---- A: save ----
        if (gamepad1.a && !prevA) {
            if (!usable) {
                message = "NOT saved: turn the turret a full 90 degrees first (it has only turned " + Math.abs(t) + " ticks).";
            } else {
                try (FileWriter w = new FileWriter(LimitsTuner.CALIBRATION_FILE)) {
                    w.write(String.format(Locale.US, "%.4f%n%s%n", tpd, METHOD));
                    mechanumtest.TURRET_TICKS_PER_DEGREE = tpd;
                    message = String.format(Locale.US, "SAVED %.2f ticks per degree. Every OpMode uses it from its next INIT.", tpd);
                    inUse = inUseText();
                    saved = true;
                    gamepad1.rumbleBlips(2);
                } catch (Exception e) {
                    message = "Could not write the file on the Control Hub.";
                }
            }
        }

        // ---- BACK: delete (back to Thursday's 4.0) ----
        if (gamepad1.back && !prevBack) {
            if (LimitsTuner.CALIBRATION_FILE.exists()) LimitsTuner.CALIBRATION_FILE.delete();
            mechanumtest.TURRET_TICKS_PER_DEGREE = mechanumtest.THURSDAY_TICKS_PER_DEGREE;
            message = "Deleted: every OpMode uses Thursday's 4.0 ticks per degree again.";
            inUse = inUseText();
        }
        prevA = gamepad1.a; prevB = gamepad1.b; prevY = gamepad1.y; prevBack = gamepad1.back;

        // ---- Driver Station ----
        telemetry.addData("Turret", "%d ticks from forward. The code says it turned %s.", t, way);
        if (Math.abs(t) > 90 && way.equals("RIGHT")) {
            telemetry.addLine("   If you actually turned it LEFT, the code's direction is backwards: tell the coder "
                    + "(TURRET_DIR in mechanumtest must flip, or auto-aim spins away).");
        }
        telemetry.addData("Press A now to save", usable ? String.format(Locale.US, "%d / 90 = %.2f ticks per degree", Math.abs(t), tpd)
                : "turn the turret 90 degrees LEFT first");
        String check = limitsCheck(savedLimits, tpd);
        if (usable && !check.isEmpty()) telemetry.addLine(check);
        telemetry.addData("Now in use", inUse);
        if (atMax || atMin) telemetry.addLine("AT THE SAVED TURRET LIMIT: go back. Turn 90 degrees to the LEFT (that side has room).");
        if (handMode) telemetry.addLine("HAND MODE: turn the turret by hand (Y = motor holds again). Mind the limits and wires.");
        if (saved && Math.abs(t) > 20) telemetry.addLine("Before STOP: hold X to bring the turret back to forward (the next INIT needs it).");
        if (!message.isEmpty()) telemetry.addLine(message);
        telemetry.addLine();
        telemetry.addLine("D-pad / stick: move | Y: hand mode | A: SAVE (exactly 90 deg LEFT) | hold X: back to forward | B: set forward | BACK: delete");
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.setPower(0);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** "4.00 ticks per degree (Thursday's)" / "7.21 ... (from the Ticks Per Degree OpMode)" ... */
    static String inUseText() {
        double v = LimitsTuner.ticksPerDegreeInUse();
        String m = LimitsTuner.calibrationMethod();
        boolean measured = LimitsTuner.isUsableMethod(m) && !Double.isNaN(LimitsTuner.loadTicksPerDegree());
        String from = measured ? "from the " + (METHOD.equals(m) ? "Ticks Per Degree OpMode" : "Limits Tuner camera check")
                : "Thursday's" + (m.isEmpty() ? "" : "; an old by-eye value on file is ignored");
        return String.format(Locale.US, "%.2f ticks per degree (%s)", v, from);
    }

    /** Compare with the saved turret limits: the turret can't turn a whole circle. */
    private static String limitsCheck(int[] lim, double tpd) {
        if (lim == null || tpd <= 0) return "";
        double range = (lim[1] - lim[0]) / tpd;
        String s = String.format(Locale.US, "With this number, the saved turret limits are %.0f degrees apart", range);
        return range > 370 ? s + ": more than a full turn, so this number and the saved limits disagree - either the 90 degrees "
                + "or the saved limits (Limits Tuner) are off." : s + ".";
    }
}
