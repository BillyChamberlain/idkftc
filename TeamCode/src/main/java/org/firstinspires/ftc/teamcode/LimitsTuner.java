package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.List;
import java.util.Locale;

/**
 * Limits Tuner: drive the turret by hand and save its soft limits. The main
 * TeleOp (mechanumtest) reads them from the Control Hub every time it INITs.
 *
 * Point the turret straight FORWARD before pressing INIT: that's zero, the same
 * as in the main TeleOp.
 *   D-pad left/right   move the turret (fixed speed)
 *   left stick X       move the turret (variable speed, for fine moves)
 *   LT                 mark where the turret is now as the LEFT limit
 *   RT                 mark where the turret is now as the RIGHT limit
 *   A                  save the limits (the main TeleOp uses them from its next INIT)
 *   B                  delete the saved limits (main TeleOp goes back to the ones in code)
 *   (Ticks per degree by turning 90 degrees: that's its own OpMode now, "Ticks Per Degree".)
 *   BACK               delete the ticks-per-degree and turn-direction files (back to Thursday's 4.0)
 *   RB                 CAMERA CHECK (the other way is the "Ticks Per Degree" OpMode): park facing any AprilTag and press RB. The turret
 *                      steps by itself and the camera measures the real ticks per degree; then turn the
 *                      robot slowly with the RIGHT STICK so it can check the turn direction. RB again
 *                      saves the results, LB throws them away (or cancels). See TurretCheck.java.
 * There are NO limits in this OpMode for D-pad/stick moves: go slowly near the ends. (The camera check
 * stays inside the saved limits.)
 */
@TeleOp(name = "Limits Tuner", group = "Tuning")
public class LimitsTuner extends OpMode {

    static final File LIMITS_FILE = new File(AppUtil.FIRST_FOLDER, "turret_limits.txt");
    static final File CALIBRATION_FILE = new File(AppUtil.FIRST_FOLDER, "turret_ticks_per_degree.txt");
    /** +1 / -1 from the camera check's turn-direction step (the main TeleOp mirrors the gyro for the aim if -1). */
    static final File HEADING_SIGN_FILE = new File(AppUtil.FIRST_FOLDER, "turret_heading_sign.txt");
    private static final double CHECK_TURN_POWER = 0.15;  // robot turning power (right stick) in step 2: slow
    // Camera check start rules: the tag must be near the middle of the picture and far enough away. (The camera
    // sits a few inches off the turret's pivot, which skews the measurement by about offset / distance.)
    private static final double CHECK_MAX_START_TX = 6.0, CHECK_MIN_DIST_IN = 90.0;
    // Without saved limits the check stays inside 80% of the code limits at the SMALLEST plausible ticks per
    // degree (4), so it can never reach a hard stop whatever the real number is.
    private static final double SAFE_TICKS_PER_DEG = 4.0;
    private static final double DPAD_POWER = 0.3;
    private static final double STICK_MAX_POWER = 0.5;

    private DcMotorEx turret;
    private Integer leftMark, rightMark;  // marked with LT/RT, not saved yet
    private int[] saved;                  // {minTicks, maxTicks} on the Control Hub, or null
    private boolean prevA, prevB, prevX, prevY, prevBack, prevLT, prevRT, prevRB, prevLB;
    private String message = "";

    // Camera check (RB): the camera, the gyro and the drive motors (only to turn the robot in step 2)
    private LimelightCam ll;
    private GoBildaPinpointDriver pinpoint;
    private DcMotor rf, rb, lf, lb;
    private final TurretCheck check = new TurretCheck(mechanumtest.TURRET_DIR);
    private final ElapsedTime clock = new ElapsedTime();
    private double lastPictureTs = Double.NaN, lastPictureSig = Double.NaN;
    private int checkTagId = -1;   // the AprilTag the check follows
    private String cachedMethod = "";  // how the ticks per degree was measured (read once, not every loop)
    private String cachedInUse = "";   // "4.00 ticks per degree (Thursday's)" ... (read once, refreshed after saves)
    private int cachedHeadingSign = 1;

    // ---------------- Saved file (used by the main TeleOp too) ----------------

    /** Saved limits as {minTicks, maxTicks} from the INIT (forward) position, or null if none/bad. */
    static int[] loadLimits() {
        if (!LIMITS_FILE.exists()) return null;
        try (BufferedReader r = new BufferedReader(new FileReader(LIMITS_FILE))) {
            String line;
            while ((line = r.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || !Character.isDigit(line.charAt(line.length() - 1))) continue;
                String[] p = line.split(",");
                if (p.length < 2) continue;
                int a = Integer.parseInt(p[0].trim()), b = Integer.parseInt(p[1].trim());
                int min = Math.min(a, b), max = Math.max(a, b);
                return (min < 0 && max > 0) ? new int[]{min, max} : null;
            }
        } catch (Exception e) {
            return null;
        }
        return null;
    }

    /** Saved ticks-per-degree from the calibration, or NaN if none/bad. */
    static double loadTicksPerDegree() {
        if (!CALIBRATION_FILE.exists()) return Double.NaN;
        try (BufferedReader r = new BufferedReader(new FileReader(CALIBRATION_FILE))) {
            double v = Double.parseDouble(r.readLine().trim());
            return (v > 0.5 && v < 40) ? v : Double.NaN;
        } catch (Exception e) {
            return Double.NaN;
        }
    }

    /** How the ticks-per-degree was measured: "camera" (RB here) or "90-degree tuner" (Ticks Per Degree OpMode),
     *  both USED; "90 degrees by eye" for an old file with no 2nd line, IGNORED (that was the 17.52); "" if none. */
    static String calibrationMethod() {
        if (!CALIBRATION_FILE.exists()) return "";
        try (BufferedReader r = new BufferedReader(new FileReader(CALIBRATION_FILE))) {
            r.readLine();
            String m = r.readLine();
            return m == null || m.trim().isEmpty() ? "90 degrees by eye" : m.trim();
        } catch (Exception e) {
            return "";
        }
    }

    /** True for a ticks-per-degree that every OpMode may use: from the Ticks Per Degree OpMode or the camera
     *  check. (An old file without a method line, from the Limits Tuner's former X/Y, is ignored: that
     *  17.52 made the aim jitter.) */
    static boolean isUsableMethod(String method) {
        return "camera".equals(method) || TicksPerDegreeTuner.METHOD.equals(method);
    }

    /** THE turret ticks per degree every OpMode uses: the saved one if usable, else Thursday's 4.0. */
    static double ticksPerDegreeInUse() {
        double v = isUsableMethod(calibrationMethod()) ? loadTicksPerDegree() : Double.NaN;
        return Double.isNaN(v) ? mechanumtest.THURSDAY_TICKS_PER_DEGREE : v;
    }

    /** The saved turn direction from the camera check: +1 normal, -1 mirrored (default +1). */
    static int loadHeadingSign() {
        if (!HEADING_SIGN_FILE.exists()) return 1;
        try (BufferedReader r = new BufferedReader(new FileReader(HEADING_SIGN_FILE))) {
            return Integer.parseInt(r.readLine().trim()) < 0 ? -1 : 1;
        } catch (Exception e) {
            return 1;
        }
    }

    static boolean saveLimits(int minTicks, int maxTicks) {
        File dir = LIMITS_FILE.getParentFile();
        if (dir != null && !dir.exists()) dir.mkdirs();
        try (FileWriter w = new FileWriter(LIMITS_FILE)) {
            w.write("min_ticks,max_ticks\n" + minTicks + "," + maxTicks + "\n");
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    // ---------------- OpMode ----------------

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "aim");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        saved = loadLimits();
        ll = new LimelightCam(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odcpu");
        pinpoint.resetPosAndIMU();  // the robot must be still for a moment (only the gyro is used here)
        rf = hardwareMap.get(DcMotor.class, "RF");
        rb = hardwareMap.get(DcMotor.class, "RB");
        lf = hardwareMap.get(DcMotor.class, "LF");
        lb = hardwareMap.get(DcMotor.class, "LB");
        rb.setDirection(DcMotorSimple.Direction.REVERSE);  // same as the main TeleOp
        for (DcMotor m : new DcMotor[]{rf, rb, lf, lb}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        cachedMethod = calibrationMethod(); cachedInUse = TicksPerDegreeTuner.inUseText(); cachedHeadingSign = loadHeadingSign();
        mechanumtest.TURRET_TICKS_PER_DEGREE = ticksPerDegreeInUse();  // the same number every OpMode uses
        telemetry.addLine("Limits Tuner: the turret must point straight FORWARD now (that's zero).");
        telemetry.addLine("Saved limits: " + describe(saved));
        telemetry.addLine("Press PLAY.");
    }

    @Override
    public void loop() {
        int t = turret.getCurrentPosition();
        double now = clock.milliseconds();
        pinpoint.update();
        double heading = pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);

        // ---- Camera: is this a NEW picture, and where is the followed tag in it? ----
        LLResult res = ll.getLatest();
        double ts = res != null ? res.getTimestamp() : Double.NaN;
        double sig = signature(res);
        boolean newPicture = res != null && (ts != lastPictureTs || sig != lastPictureSig);
        if (res != null) { lastPictureTs = ts; lastPictureSig = sig; }
        double tagTx = newPicture ? tagTx(res, checkTagId) : Double.NaN;

        // ---- RB: start the camera check / save its results.  LB: stop / throw away ----
        boolean rbNow = gamepad1.right_bumper, lbNow = gamepad1.left_bumper;
        int centerTag = centerTagId(res);
        double centerTx = tagTx(res, centerTag), centerDist = tagDistanceIn(res, centerTag);
        if (rbNow && !prevRB) {
            if (check.stage == TurretCheck.Stage.DONE) {
                check.saved(saveCheck(true));
            } else if (check.stage == TurretCheck.Stage.HEADING) {
                check.saved(saveCheck(false));  // save step 1 only (the turn direction stays as it was)
            } else if (check.stage != TurretCheck.Stage.SCALE) {
                if (centerTag < 0) {
                    message = "Camera check: no AprilTag in view. Park facing one and press RB.";
                } else if (Math.abs(centerTx) > CHECK_MAX_START_TX) {
                    message = String.format(Locale.US, "Camera check: tag %d is %.0f deg %s of the middle. D-pad the turret until it is "
                            + "within %.0f deg, then RB.", centerTag, Math.abs(centerTx), centerTx > 0 ? "right" : "left", CHECK_MAX_START_TX);
                } else if (!Double.isNaN(centerDist) && centerDist < CHECK_MIN_DIST_IN) {
                    message = String.format(Locale.US, "Camera check: the tag is %.0f in away. Back up to at least %.0f in (about 8 ft) "
                            + "for an accurate measurement, then RB.", centerDist, CHECK_MIN_DIST_IN);
                } else {
                    checkTagId = centerTag;
                    check.start(now, t, safeLow(), safeHigh());
                    message = "Camera check started on tag " + checkTagId + ".";
                }
            }
        }
        if (lbNow && !prevLB && check.stage != TurretCheck.Stage.IDLE) {
            check.discard();
            message = "Camera check stopped; nothing saved.";
        }
        prevRB = rbNow; prevLB = lbNow;
        boolean checkRunning = check.stage == TurretCheck.Stage.SCALE || check.stage == TurretCheck.Stage.HEADING;
        // While the check runs or has results waiting, the other buttons are locked (an accidental A/B/BACK
        // would overwrite or delete saved files)
        boolean buttonsFree = check.stage == TurretCheck.Stage.IDLE || check.stage == TurretCheck.Stage.FAILED;
        double checkPower = check.update(now, t, heading, newPicture, tagTx);

        // ---- Turret: the check drives it while it runs; otherwise the D-pad / left stick ----
        double power = 0;
        if (checkRunning) power = checkPower;
        else if (gamepad1.dpad_right) power = DPAD_POWER;
        else if (gamepad1.dpad_left) power = -DPAD_POWER;
        else if (Math.abs(gamepad1.left_stick_x) > 0.05) power = gamepad1.left_stick_x * STICK_MAX_POWER;
        turret.setPower(power);

        // ---- Robot turning (only in step 2 of the check): right stick, slowly ----
        double turn = check.stage == TurretCheck.Stage.HEADING ? -gamepad1.right_stick_x * CHECK_TURN_POWER : 0;
        if (Math.abs(gamepad1.right_stick_x) < 0.05) turn = 0;
        rf.setPower(turn); rb.setPower(turn); lf.setPower(-turn); lb.setPower(-turn);

        boolean lt = gamepad1.left_trigger > 0.5, rt = gamepad1.right_trigger > 0.5;
        if (buttonsFree && lt && !prevLT) { leftMark = t; message = "LEFT limit marked at " + deg(t) + ". Press A to save."; gamepad1.rumble(150); }
        if (buttonsFree && rt && !prevRT) { rightMark = t; message = "RIGHT limit marked at " + deg(t) + ". Press A to save."; gamepad1.rumble(150); }

        if (buttonsFree && gamepad1.a && !prevA) {
            int left = leftMark != null ? leftMark : currentLeft();
            int right = rightMark != null ? rightMark : currentRight();
            if ((left < 0 && right > 0) || (left > 0 && right < 0)) {
                int min = Math.min(left, right), max = Math.max(left, right);
                if (saveLimits(min, max)) {
                    saved = new int[]{min, max};
                    leftMark = null;
                    rightMark = null;
                    message = "SAVED. The main TeleOp uses these from its next INIT.";
                    gamepad1.rumbleBlips(2);
                } else {
                    message = "Could not write the file on the Control Hub.";
                }
            } else {
                message = "NOT saved: the left and right limits must be on opposite sides of forward.";
            }
        }
        if (buttonsFree && gamepad1.b && !prevB) {
            if (LIMITS_FILE.exists()) LIMITS_FILE.delete();
            saved = null;
            message = "Saved limits deleted: the main TeleOp uses the limits in code.";
        }
        if (buttonsFree && ((gamepad1.x && !prevX) || (gamepad1.y && !prevY))) {
            // The 90-degree ticks-per-degree measurement moved to its own OpMode
            message = "Ticks per degree: use the \"Ticks Per Degree\" OpMode (turn 90 degrees, press A), or RB here (camera check).";
        }
        if (buttonsFree && gamepad1.back && !prevBack) {
            if (CALIBRATION_FILE.exists()) CALIBRATION_FILE.delete();
            if (HEADING_SIGN_FILE.exists()) HEADING_SIGN_FILE.delete();
            mechanumtest.TURRET_TICKS_PER_DEGREE = mechanumtest.THURSDAY_TICKS_PER_DEGREE;
            cachedMethod = calibrationMethod(); cachedInUse = TicksPerDegreeTuner.inUseText(); cachedHeadingSign = loadHeadingSign();
            message = "Calibration and turn direction deleted: the main TeleOp uses Thursday's 4.0 ticks per degree, normal turn direction.";
        }
        prevA = gamepad1.a; prevB = gamepad1.b; prevLT = lt; prevRT = rt;
        prevX = gamepad1.x; prevY = gamepad1.y; prevBack = gamepad1.back;

        telemetry.addLine("CAMERA CHECK: " + check.status);
        if (buttonsFree) {  // live: can RB start now?
            telemetry.addLine(centerTag < 0 ? "   Camera: no AprilTag seen."
                    : String.format(Locale.US, "   Camera: tag %d, %.1f deg %s of the middle, %s away: %s", centerTag, Math.abs(centerTx),
                    centerTx > 0 ? "right" : "left", Double.isNaN(centerDist) ? "?" : String.format(Locale.US, "%.0f in", centerDist),
                    Math.abs(centerTx) <= CHECK_MAX_START_TX && (Double.isNaN(centerDist) || centerDist >= CHECK_MIN_DIST_IN)
                            ? "OK to press RB" : "not yet (middle within 6 deg, at least 90 in away)"));
        }
        String results = check.resultText();
        if (!results.isEmpty()) telemetry.addLine(results);
        telemetry.addData("Turret now", "%d ticks = %s  (%s; turn direction %s)", t, deg(t),
                cachedInUse, cachedHeadingSign < 0 ? "MIRRORED (saved)" : "normal");
        telemetry.addLine();
        telemetry.addData("LEFT  (LT marks)", leftMark != null ? deg(leftMark) + "  (not saved)" : deg(currentLeft()));
        telemetry.addData("RIGHT (RT marks)", rightMark != null ? deg(rightMark) + "  (not saved)" : deg(currentRight()));
        telemetry.addData("Main TeleOp uses", describe(saved));
        if (!message.isEmpty()) telemetry.addLine(message);
        telemetry.addLine();
        telemetry.addLine("D-pad / left stick move | LT left | RT right | A save | B delete saved");
        telemetry.addLine("Ticks per degree: \"Ticks Per Degree\" OpMode (turn 90 deg, A) or RB camera check here");
        telemetry.addLine("RB: camera check (best way to measure ticks per degree) | LB: stop it");
        telemetry.addLine("No limits here: go slowly near the ends.");
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.setPower(0);
        for (DcMotor m : new DcMotor[]{rf, rb, lf, lb}) if (m != null) m.setPower(0);
    }

    // ---------------- Camera check helpers ----------------

    /** Lowest / highest tick count the camera check may drive the turret to (ticks from forward at INIT). */
    private double safeLow() {
        if (saved != null) return saved[0] + Math.max(60, 0.1 * Math.abs(saved[0]));
        double right = 0.8 * mechanumtest.TURRET_RIGHT_LIMIT_DEG * SAFE_TICKS_PER_DEG;
        double left = 0.8 * mechanumtest.TURRET_LEFT_LIMIT_DEG * SAFE_TICKS_PER_DEG;
        return mechanumtest.TURRET_DIR > 0 ? -left : -right;
    }

    private double safeHigh() {
        if (saved != null) return saved[1] - Math.max(60, 0.1 * Math.abs(saved[1]));
        double right = 0.8 * mechanumtest.TURRET_RIGHT_LIMIT_DEG * SAFE_TICKS_PER_DEG;
        double left = 0.8 * mechanumtest.TURRET_LEFT_LIMIT_DEG * SAFE_TICKS_PER_DEG;
        return mechanumtest.TURRET_DIR > 0 ? right : left;
    }

    /** Save the camera check: ticks per degree (marked "camera") and, if wanted, the turn direction. */
    private String saveCheck(boolean withTurnDirection) {
        StringBuilder out = new StringBuilder();
        if (!Double.isNaN(check.ticksPerDeg) && check.ticksPerDeg > 0.5 && check.ticksPerDeg < 40) {
            try (FileWriter w = new FileWriter(CALIBRATION_FILE)) {
                w.write(String.format(Locale.US, "%.4f%ncamera%n", check.ticksPerDeg));
                mechanumtest.TURRET_TICKS_PER_DEGREE = check.ticksPerDeg;
                out.append(String.format(Locale.US, "SAVED %.2f ticks per degree. ", check.ticksPerDeg));
            } catch (Exception e) {
                out.append("Could not save ticks per degree. ");
            }
        }
        if (withTurnDirection && check.headingSign != 0) {
            try (FileWriter w = new FileWriter(HEADING_SIGN_FILE)) {
                w.write(Integer.toString(check.headingSign));
                out.append("SAVED turn direction: ").append(check.headingSign > 0 ? "normal." : "mirrored.");
            } catch (Exception e) {
                out.append("Could not save the turn direction.");
            }
        } else if (withTurnDirection) {
            // Unclear: don't leave an old "mirrored" in use without anyone knowing
            if (HEADING_SIGN_FILE.exists()) HEADING_SIGN_FILE.delete();
            out.append("Turn direction unclear: set back to normal (redo step 2 to measure it).");
        } else {
            out.append("Turn direction not measured: kept as it was (").append(loadHeadingSign() < 0 ? "mirrored" : "normal").append(").");
        }
        cachedMethod = calibrationMethod(); cachedInUse = TicksPerDegreeTuner.inUseText(); cachedHeadingSign = loadHeadingSign();
        out.append(" The main TeleOp uses this from its next INIT.");
        gamepad1.rumbleBlips(2);
        return "SAVED. " + out.toString();
    }

    /** The AprilTag closest to the middle of the picture, or -1. */
    private static int centerTagId(LLResult r) {
        if (r == null || !r.isValid()) return -1;
        List<LLResultTypes.FiducialResult> f = r.getFiducialResults();
        if (f == null) return -1;
        int best = -1;
        double bestTx = Double.POSITIVE_INFINITY;
        for (LLResultTypes.FiducialResult x : f) {
            if (Math.abs(x.getTargetXDegrees()) < bestTx) { bestTx = Math.abs(x.getTargetXDegrees()); best = x.getFiducialId(); }
        }
        return best;
    }

    /** Distance (inches) from the camera to tag id, from the Limelight's 3D pose, or NaN if unknown. */
    private static double tagDistanceIn(LLResult r, int id) {
        if (r == null || !r.isValid() || id < 0) return Double.NaN;
        List<LLResultTypes.FiducialResult> f = r.getFiducialResults();
        if (f == null) return Double.NaN;
        for (LLResultTypes.FiducialResult x : f) {
            if (x.getFiducialId() != id) continue;
            try {
                org.firstinspires.ftc.robotcore.external.navigation.Position p = x.getTargetPoseCameraSpace().getPosition()
                        .toUnit(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
                double d = Math.hypot(p.x, p.z);
                return d > 1 ? d : Double.NaN;
            } catch (Exception e) {
                return Double.NaN;
            }
        }
        return Double.NaN;
    }

    /** tx of tag id in this picture, or NaN if it isn't there. */
    private static double tagTx(LLResult r, int id) {
        if (r == null || !r.isValid() || id < 0) return Double.NaN;
        List<LLResultTypes.FiducialResult> f = r.getFiducialResults();
        if (f == null) return Double.NaN;
        for (LLResultTypes.FiducialResult x : f) if (x.getFiducialId() == id) return x.getTargetXDegrees();
        return Double.NaN;
    }

    /** What the camera saw, as one number: the same picture sent again gives the same number. */
    private static double signature(LLResult r) {
        if (r == null) return Double.NaN;
        List<LLResultTypes.FiducialResult> f = r.getFiducialResults();
        double s = 0;
        if (f != null) for (LLResultTypes.FiducialResult x : f) s += x.getFiducialId() + x.getTargetXDegrees() * 1.0001 + x.getTargetYDegrees() * 0.37;
        return s;
    }

    // ---------------- Helpers ----------------

    /** Left limit in use now (saved file, else the one in code), in encoder ticks. */
    private int currentLeft() {
        if (saved != null) return mechanumtest.TURRET_DIR > 0 ? saved[0] : saved[1];
        return (int) Math.round(-mechanumtest.TURRET_DIR * mechanumtest.TURRET_LEFT_LIMIT_DEG * mechanumtest.TURRET_TICKS_PER_DEGREE);
    }

    private int currentRight() {
        if (saved != null) return mechanumtest.TURRET_DIR > 0 ? saved[1] : saved[0];
        return (int) Math.round(mechanumtest.TURRET_DIR * mechanumtest.TURRET_RIGHT_LIMIT_DEG * mechanumtest.TURRET_TICKS_PER_DEGREE);
    }

    /** "35 deg right" / "280 deg left" from encoder ticks. */
    static String deg(int ticks) {
        double d = ticks / mechanumtest.TURRET_TICKS_PER_DEGREE * mechanumtest.TURRET_DIR;
        return String.format(Locale.US, "%.0f deg %s", Math.abs(d), d >= 0 ? "right" : "left");
    }

    static String describe(int[] limits) {
        if (limits == null) return "the ones in code (nothing saved)";
        return deg(limits[0]) + " to " + deg(limits[1]) + " (saved by Limits Tuner)";
    }
}
