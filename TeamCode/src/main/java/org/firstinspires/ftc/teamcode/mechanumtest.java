package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.List;
import java.util.Locale;

/*
 * The original TeleOp (Will's code), with these ADDED (each one is marked "ADDED" below):
 *   - AUTO AIM: the turret points at your alliance's hive by itself (TurretAimer.java does the math).
 *       INIT: X = BLUE, B = RED.  D-pad DOWN = switch color, D-pad UP = the other hive of your color.
 *       D-pad LEFT/RIGHT still moves the turret by hand (as before); let go and auto aim takes over again.
 *       LT + RT together = "the robot is in the start corner": resets the odometry.
 *   - Turret limits / ticks per degree from the Limits Tuner and Ticks Per Degree OpModes; aim gains in
 *     TurretTuning (live in Panels).
 *   - Clencher: Y = deployed <-> retracted (positions from the Clencher Tuner).
 *     RT = shot: clencher retracts, from halfway through its move the intake + transfer run for 2 s,
 *     then it deploys again.   LT tap = intake only on/off, LT hold = intake backwards.
 *   - Headlights: turretLight on when the turret is LOCKED, flyLight on when the flywheel is at speed.
 *   - Strafe stick fixed (it slid the robot the opposite way).
 *   - Turned off (commented out): the old "Limelight -> Pinpoint position override". The camera rides on the
 *     turret, so it broke the auto aim's odometry. X/Y on the Driver Station now count from INIT (or LT+RT).
 *   - The hood isn't moved in this TeleOp (RT/LT used to move it; they are the shot and the intake now).
 */
@TeleOp
public class mechanumtest extends OpMode {
    private DcMotor RF;
    private DcMotor LF;
    private DcMotor RB;
    private DcMotor LB;

    private DcMotor intake;

    private DcMotor trans;

    private DcMotorEx turret_shoot;
    private DcMotorEx turret_aim;
    final double MAX_TICKS_PER_SECOND = 2400.0;
    private double bpow = 0.5;
    boolean in_run;
    boolean out_run;

    public int ticks;

    private final float flyconstant = 1f; // TODO: NEEDS TO BE CHANGED

    private final float flywheel_dia = 0.1202436f;

    //TODO: Change these vals to match field size
    private final double slowspeedlen = 1;
    private final double slowspeedspeed = 3000 / 60.0;


    private final double medspeedlen = 2;
    private final double medspeedspeed = 4000 / 60.0;

    private final double fastspeedlen = 3;
    private final double fastspeedspeed = 5000 / 60.0;

    private boolean prevA;
    private boolean prevB;
    private boolean prevY;
    private boolean prevLB;
    private boolean prevRB;
    private boolean prevLT;
    private boolean prevRT;

    private boolean trans_open;   // ADDED meaning: true = clencher DEPLOYED, false = RETRACTED

    private Servo transfer1;
    private CRServo hood1;
    private CRServo hood2;

    private LLResult results;
    private double tx;
    LimelightCam ll;
    int maxticks = 0;
    int minticks = 0;
    int initticks;
    boolean aimrightpositive = true;
    boolean maxed = false;
    boolean istx = false;
    double kp = 0.1;

    private GoBildaPinpointDriver pinpoint;

    // ======================= ADDED: AUTO AIM SETTINGS =======================
    // +1 if positive motor power turns the turret RIGHT (D-pad right does).
    static final int TURRET_DIR = +1;
    // Turret motor ticks per degree of turret turn. Thursday's measured value is 4.0; the "Ticks Per
    // Degree" OpMode (turn 90 deg, press A) or the Limits Tuner camera check (RB) replace it.
    static final double THURSDAY_TICKS_PER_DEGREE = 4.0;
    static double TURRET_TICKS_PER_DEGREE = THURSDAY_TICKS_PER_DEGREE;
    // The original turret limits (ticks from where the turret points at INIT: point it FORWARD first).
    // The Limits Tuner can save new ones; if it has, those are used instead.
    private static final int OG_RIGHT_LIMIT_TICKS = 580, OG_LEFT_LIMIT_TICKS = 770;
    // The same limits in degrees at 4.0 ticks/deg (the Limits Tuner shows them).
    static final double TURRET_RIGHT_LIMIT_DEG = OG_RIGHT_LIMIT_TICKS / THURSDAY_TICKS_PER_DEGREE;
    static final double TURRET_LEFT_LIMIT_DEG = OG_LEFT_LIMIT_TICKS / THURSDAY_TICKS_PER_DEGREE;
    private static final double TURRET_MAX_TICKS_PER_SEC = 2800.0;  // turret motor top speed
    private static final double TURRET_LIMIT_SLOWDOWN = 10.0;       // slow down near a limit (deg/s per deg left)
    private static final double LOCKED_TOLERANCE_DEG = 2.0;
    private static final double SEARCH_SWEEP_DEG_PER_SEC = 45.0;    // turret speed while looking for the hive
    private static final double MAX_FRAME_AGE_MS = 250;             // ignore older camera results

    // Odometry pods on the goBILDA Pinpoint, measured with the Pedro Pinpoint tuner
    private static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
    private static final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    private static final double X_POD_OFFSET_MM = 1.1514677212932918 * 25.4, Y_POD_OFFSET_MM = -2.941031869002215 * 25.4;

    // BIOBUZZ hives: 4 AprilTags each (base..base+3)
    private static final int RED_SCORING_BASE = 30, RED_AUDIENCE_BASE = 34, BLUE_AUDIENCE_BASE = 38, BLUE_SCORING_BASE = 42;
    private static final int[] CELL_BASES = {RED_SCORING_BASE, RED_AUDIENCE_BASE, BLUE_AUDIENCE_BASE, BLUE_SCORING_BASE};
    private static final int[] RED_CELLS = {0, 1};
    private static final int[] BLUE_CELLS = {3, 2};
    private static final int NO_GOAL = 4;   // aimer that never sees anything (until a hive of our color is seen)

    // ======================= ADDED: CLENCHER, RT, LT, LIGHTS SETTINGS =======================
    // Default clencher positions (the Clencher Tuner saves better ones): SHUT = retracted, OPEN = deployed
    static final double CLENCHER_SHUT_POS = 0.34;
    static final double CLENCHER_OPEN_POS = Math.max(0, CLENCHER_SHUT_POS - 90.0 / 270.0);
    private static final double CLENCHER_MOVE_MS = 400;   // time for the clencher's full move (a guess)
    private static final double RT_FEED_MS = 2000;        // RT: intake + transfer run this long
    private static final double RT_CHORD_WAIT_MS = 120;   // RT waits for LT (LT+RT = position reset)
    private static final double LT_HOLD_MS = 300;         // LT held longer than this = intake backwards
    private static final double LIGHT_ON = 1.0, LIGHT_OFF = 0.0;   // headlight servo positions (swap if backwards)

    // ======================= ADDED: working state =======================
    private final ElapsedTime clock = new ElapsedTime();
    private boolean isRedAlliance = true;
    private final TurretAimer[] aimers = new TurretAimer[5];
    private TurretAimer aimer;
    private int activeGoal = -1;            // the hive we aim at (-1 until one of our color is seen)
    private boolean manualHold, searching, prevUp, prevDown, resetChord;
    private int sweepDir = 1, searchSign = 1;
    private double searchAmpDeg;
    private int headingSign = 1;            // -1 only if the Limits Tuner camera check found it mirrored
    private double lastGoodX = 0, lastGoodY = 0;   // last good Pinpoint x/y (one bad read mustn't look like a jump)
    private double ltDownSinceMs = Double.NaN, rtDownSinceMs = Double.NaN;   // for the LT+RT chord
    private boolean turretScaleMeasured;
    private double lastPictureTs = Double.NaN, lastPictureSig = Double.NaN;
    private String tagsInView = "none";
    private final TurretAimer.ClusterFit[] clusterFits = {
            new TurretAimer.ClusterFit(), new TurretAimer.ClusterFit(),
            new TurretAimer.ClusterFit(), new TurretAimer.ClusterFit()};
    private double appliedMotorP = Double.NaN, appliedMotorI, appliedMotorD, appliedMotorF;
    private double lastTurretCmdTps = 0;
    private double aTransPower = 1;         // A: transfer power (the original toggleIntake(1))
    private double clencherRetracted = CLENCHER_SHUT_POS, clencherDeployed = CLENCHER_OPEN_POS;
    private double lastClencherSendMs = Double.NEGATIVE_INFINITY;
    private double rtPendingMs = Double.NaN, feedStartMs = Double.NaN, feedEndMs = Double.NaN;
    private boolean ltIntakeOn, ltReverse, ltChord;
    private double ltPressedMs;
    private Servo turretLight, flyLight;
    private double turretLightUntilMs = Double.NEGATIVE_INFINITY;
    private boolean flyLightOn;
    private double lastLoopMs = Double.NaN;

    public void init(){
        //Drive
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        ll = new LimelightCam(hardwareMap);
        ll.start();

        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        //Other mechs that shouldn't need to be changed
        intake = hardwareMap.get(DcMotor.class, "intake");
        trans = hardwareMap.get(DcMotor.class, "transfer");
        turret_shoot = hardwareMap.get(DcMotorEx.class, "shoot");
        turret_shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret_aim = hardwareMap.get(DcMotorEx.class, "aim");
//

        transfer1 = hardwareMap.get(Servo.class, "t1");
        hood1 = hardwareMap.get(CRServo.class, "hood1");
        hood2 = hardwareMap.get(CRServo.class, "hood2");

        turret_shoot.setVelocityPIDFCoefficients(35.5, 0.0, 1.0, 11.7);
        turret_aim.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret_aim.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ticks = turret_aim.getCurrentPosition();
        initticks = ticks;
        minticks = ticks - OG_LEFT_LIMIT_TICKS;
        maxticks = ticks + OG_RIGHT_LIMIT_TICKS;


        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odcpu");

        // ADDED: the odometry pods' settings, so x/y are right for the auto aim
        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(POD_TYPE);
        pinpoint.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);
        pinpoint.resetPosAndIMU();   // robot must be still for a moment
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));

        // ADDED: auto aim setup
        turret_aim.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        applyTurretMotorPidf();
        TURRET_TICKS_PER_DEGREE = LimitsTuner.ticksPerDegreeInUse();   // Ticks Per Degree OpMode / camera check, else 4.0
        turretScaleMeasured = LimitsTuner.isUsableMethod(LimitsTuner.calibrationMethod())
                && !Double.isNaN(LimitsTuner.loadTicksPerDegree());
        int[] saved = LimitsTuner.loadLimits();                         // Limits Tuner, else the original limits
        if (saved != null) {
            minticks = ticks + saved[0];
            maxticks = ticks + saved[1];
        }
        headingSign = LimitsTuner.loadHeadingSign();
        for (int i = 0; i < aimers.length; i++) aimers[i] = new TurretAimer(turretDegrees(minticks), turretDegrees(maxticks));
        aimer = aimers[NO_GOAL];

        // ADDED: clencher positions from the Clencher Tuner; headlights (no crash if not configured)
        double[] c = ClencherTuner.positionsInUse();   // {retracted, deployed}
        clencherRetracted = c[0];
        clencherDeployed = c[1];
        turretLight = hardwareMap.tryGet(Servo.class, "turretLight");
        flyLight = hardwareMap.tryGet(Servo.class, "flyLight");
        if (turretLight != null) turretLight.setPosition(LIGHT_OFF);
        if (flyLight != null) flyLight.setPosition(LIGHT_OFF);
    }

    // ADDED: pick the alliance during INIT (X = blue, B = red)
    @Override
    public void init_loop(){
        if (gamepad1.x) isRedAlliance = false;
        if (gamepad1.b) isRedAlliance = true;
        telemetry.addData("Alliance", isRedAlliance ? "RED  (X = blue)" : "BLUE  (B = red)");
        telemetry.addData("Turret", "zeroed where it pointed when INIT was pressed: if it wasn't forward, STOP, point it forward, INIT again");
        telemetry.addData("Turret limits", "%d to %d ticks (%s), %.2f ticks/deg (%s)", minticks - initticks, maxticks - initticks,
                LimitsTuner.loadLimits() != null ? "Limits Tuner" : "original", TURRET_TICKS_PER_DEGREE,
                turretScaleMeasured ? "measured" : "Thursday's 4.0");
        telemetry.update();
    }

    // ADDED: hold the clencher RETRACTED from PLAY (so the first Y press moves it)
    @Override
    public void start(){
        clock.reset();
        trans_open = false;
        transfer1.setPosition(clencherRetracted);
        lastClencherSendMs = clock.milliseconds();
    }

    public void loop(){
        double now = clock.milliseconds();   // ADDED
        double dt = Double.isNaN(lastLoopMs) ? 0.02 : Math.max(0.001, (now - lastLoopMs) / 1000.0);
        lastLoopMs = now;
        pinpoint.update();
        telemetry.addData("maxed?", maxed);
        ticks = turret_aim.getCurrentPosition();
        telemetry.addData("ticks", ticks);
        results = ll.getVars(telemetry);
        boolean tagFound = false;

// 1. Properly search for Tag 36 without overwriting the result
        // ADDED (turned off): this put the Limelight's camera pose into the Pinpoint every frame. The camera
        // rides on the turret, so its "robot" heading includes the turret angle, and the jump breaks the auto
        // aim's odometry. The auto aim uses the camera directly instead.
//        Pose3D botpose = results.getBotpose();
//        if(botpose !=null && results.isValid()) {
//
//            double fieldX_meters = botpose.getPosition().x;
//            double fieldY_meters = botpose.getPosition().y;
//            double robotYaw_deg = botpose.getOrientation().getYaw();
//
//            // C. HARD OVERRIDE: Inject Limelight absolute coordinates into the Pinpoint
//            pinpoint.setPosition(new Pose2D(
//                    DistanceUnit.METER,
//                    fieldX_meters,
//                    fieldY_meters,
//                    AngleUnit.DEGREES,
//                    robotYaw_deg
//            ));
//        }
        Pose2D currentPose = pinpoint.getPosition();

        telemetry.addData("X Position (in)", currentPose.getX(DistanceUnit.INCH));
        telemetry.addData("Y Position (in)", currentPose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)", currentPose.getHeading(AngleUnit.DEGREES));

        // ADDED note: Will's original sweep idea, left here turned off. The ADDED auto aim below replaces it
        // (it sweeps between the same limits when no hive is seen). aimrightpositive / kp / tagFound belong to it.
// 2. Update sweep direction based on soft limits
//        if (ticks <= minticks) {
//            aimrightpositive = true; // Reached left limit, sweep right
//        } else if (ticks >= maxticks) {
//            aimrightpositive = false; // Reached right limit, sweep left
//        }
//
//// 3. Control logic
//        if (!tagFound) {
//            // Tag NOT found: Sweep back and forth within tick limits
//            if (aimrightpositive) {
//                turret_aim.setPower(0.7);
//            } else {
//                turret_aim.setPower(-0.7);
//            }
//        } else {
//            // Tag FOUND: Center target between -12 and -4 degrees
//                // Target is to the right of target window -> turn towards it
//                turret_aim.setPower((aimrightpositive? 1 : -1) * kp * (tx-8));
//
//        }

        // ======================= ADDED: AUTO AIM, where is the hive? =======================
        double turretDeg = turretDegrees(ticks);
        double headingDeg = pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);   // doesn't jump at 180
        boolean odometryOk = pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
        double robotX = currentPose.getX(DistanceUnit.INCH), robotY = currentPose.getY(DistanceUnit.INCH);
        odometryOk = odometryOk && !Double.isNaN(robotX) && !Double.isNaN(robotY);
        if (odometryOk) { lastGoodX = robotX; lastGoodY = robotY; }
        for (TurretAimer a : aimers) {
            a.kP = TurretTuning.aimKP;   // gains: TurretTuning (live in Panels)
            a.kI = TurretTuning.aimKI;
            a.deadbandDeg = TurretTuning.deadbandDeg;
            a.fastestWay = TurretTuning.fastestWay;
            a.maxSpeedDegPerSec = Math.min(TurretTuning.maxAimSpeedDegPerSec, 0.9 * TURRET_MAX_TICKS_PER_SEC / TURRET_TICKS_PER_DEGREE);
            a.useOdometry = odometryOk;
            a.record(now, turretDeg, headingSign * headingDeg, lastGoodX, headingSign * lastGoodY);
        }
        applyTurretMotorPidf();

        // LT + RT pressed TOGETHER (within 0.12 s, not during a shot): the robot is in the start corner ->
        // reset the odometry (remembered hives are kept)
        boolean ltNow = gamepad1.left_trigger > 0.5, rtNow = gamepad1.right_trigger > 0.5;
        if (!ltNow) ltDownSinceMs = Double.NaN; else if (Double.isNaN(ltDownSinceMs)) ltDownSinceMs = now;
        if (!rtNow) rtDownSinceMs = Double.NaN; else if (Double.isNaN(rtDownSinceMs)) rtDownSinceMs = now;
        boolean bothTriggers = ltNow && rtNow && Math.abs(ltDownSinceMs - rtDownSinceMs) <= RT_CHORD_WAIT_MS
                && Double.isNaN(feedStartMs);
        if (bothTriggers && !resetChord) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            for (TurretAimer a : aimers) a.resetMotion();
            gamepad1.rumbleBlips(3);
        }
        resetChord = bothTriggers;

        // Each NEW camera picture (the SDK sends the same picture several times) tells every hive it shows
        // where that hive is
        double pictureTs = results != null ? results.getTimestamp() : Double.NaN;
        double pictureSig = pictureSignature(results);
        boolean newPicture = results != null && (pictureTs != lastPictureTs || pictureSig != lastPictureSig);
        if (results != null) { lastPictureTs = pictureTs; lastPictureSig = pictureSig; }
        Target[] seen = findTargets(results, now);
        for (int i = 0; i < seen.length; i++) {
            if (seen[i] != null && newPicture) aimers[i].seeTarget(now, seen[i].tx, seen[i].frameAgeMs, seen[i].distanceIn);
        }

        // D-pad DOWN = switch color (red/blue); D-pad UP = the other hive of our color
        if (gamepad1.dpad_down && !prevDown) {
            isRedAlliance = !isRedAlliance;
            activeGoal = -1;
            for (int g : colorCells()) if (activeGoal < 0 && aimers[g].isTracking(now)) activeGoal = g;
            gamepad1.rumbleBlips(isRedAlliance ? 1 : 2);
        }
        prevDown = gamepad1.dpad_down;
        if (activeGoal < 0) activeGoal = bestOf(seen, colorCells());   // the first hive of our color we see
        if (gamepad1.dpad_up && !prevUp) {
            int[] cells = colorCells();
            activeGoal = activeGoal == cells[0] ? cells[1] : cells[0];
            gamepad1.rumbleBlips(2);
        }
        prevUp = gamepad1.dpad_up;
        aimer = aimers[activeGoal >= 0 ? activeGoal : NO_GOAL];
        Target target = activeGoal >= 0 ? seen[activeGoal] : null;
        boolean targetVisible = target != null;
        boolean tracking = aimer.isTracking(now);
        double aimErrorDeg = tracking ? aimer.errorDeg(now) : 0;
        boolean parked = tracking && aimer.isParked(now);
        // LOCKED only when the camera itself sees the hive in the middle
        boolean locked = tracking && !parked && targetVisible
                && Math.abs(target.tx) <= LOCKED_TOLERANCE_DEG + 1.0 && Math.abs(aimErrorDeg) <= LOCKED_TOLERANCE_DEG;

        // ======================= ADDED: LT and RT =======================
        // LT (intake only): tap = on/off, hold = backwards while held. (LT+RT = position reset, not a tap.)
        boolean ltDown = gamepad1.left_trigger > 0.5;
        if (ltDown && !prevLT) { ltPressedMs = now; ltChord = false; }
        if (bothTriggers) ltChord = true;   // part of the LT+RT reset, not an LT tap
        ltReverse = ltDown && !ltChord && now - ltPressedMs >= LT_HOLD_MS;
        if (!ltDown && prevLT && !ltChord && now - ltPressedMs < LT_HOLD_MS) ltIntakeOn = !ltIntakeOn;
        prevLT = ltDown;
        // RT shot: retract the clencher; from HALFWAY through its move, intake + transfer run RT_FEED_MS;
        // then the clencher deploys again
        boolean rtDown = gamepad1.right_trigger > 0.5;
        if (rtDown && !prevRT) rtPendingMs = now;
        if (bothTriggers) rtPendingMs = Double.NaN;   // LT+RT = reset, not a shot (letting go of RT doesn't cancel)
        prevRT = rtDown;
        if (!Double.isNaN(rtPendingMs) && now - rtPendingMs >= RT_CHORD_WAIT_MS) {
            rtPendingMs = Double.NaN;
            double moveMs = trans_open ? CLENCHER_MOVE_MS : 0;
            trans_open = false;
            transfer1.setPosition(clencherRetracted);
            lastClencherSendMs = now;
            feedStartMs = now + moveMs / 2;
            feedEndMs = feedStartMs + RT_FEED_MS;
        }
        boolean feedActive = !Double.isNaN(feedStartMs) && now >= feedStartMs && now < feedEndMs;
        if (!Double.isNaN(feedStartMs) && now >= feedEndMs) {   // the feed is over: deploy again
            feedStartMs = Double.NaN;
            trans_open = true;
            transfer1.setPosition(clencherDeployed);
            lastClencherSendMs = now;
        }

        if(gamepad1.a && !prevA){
            toggleIntake(1);
        }
        prevA = gamepad1.a;
        // ADDED: intake + transfer are set in ONE place, because A, LT and RT all use them
        intake.setPower(ltReverse ? 1.0 : ((in_run || ltIntakeOn || feedActive) ? -1.0 : 0.0));
        trans.setPower(feedActive ? -1.0 : (in_run ? -1 * aTransPower : 0.0));

        double vertical = -gamepad1.left_stick_y;
        double horizontal = -gamepad1.left_stick_x;   // ADDED fix: was +, which slid the robot the wrong way
        double pivot = -gamepad1.right_stick_x;

        //Change + and - on this if necessary
        RF.setPower(pivot + (vertical + horizontal));
        RB.setPower(pivot + (vertical - horizontal));
        LF.setPower(-pivot + (vertical - horizontal));
        LB.setPower(-pivot + (vertical + horizontal));

        if(gamepad1.b && !prevB){
            out_run = !out_run;
        }
        prevB = gamepad1.b;
        telemetry.addData("Encoder Counts", turret_shoot.getCurrentPosition());
        if(out_run) {
            turret_shoot.setVelocity(MAX_TICKS_PER_SECOND * bpow);
            telemetry.addData("Alleged velocity", MAX_TICKS_PER_SECOND * bpow);
        }
        else turret_shoot.setPower(0);
        if(gamepad1.left_bumper && !prevLB){
            bpow -= 0.05;
        }
        prevLB = gamepad1.left_bumper;
        if(gamepad1.right_bumper && !prevRB){
            bpow += 0.05;
        }
        prevRB = gamepad1.right_bumper;
        telemetry.addData("Supposed fly speed", bpow);

        // ======================= ADDED: headlights =======================
        // turretLight: on while LOCKED (held 0.2 s so it doesn't blink). flyLight: flywheel within 8% of its
        // speed (off again only past 12%, so it doesn't flicker).
        if (locked) turretLightUntilMs = now + 200;
        double flyTarget = MAX_TICKS_PER_SECOND * bpow;
        double flyOff = flyTarget > 0 ? Math.abs(turret_shoot.getVelocity() - flyTarget) / flyTarget : 1;
        if (!out_run || flyTarget <= 0) flyLightOn = false;
        else if (flyOff <= 0.08) flyLightOn = true;
        else if (flyOff > 0.12) flyLightOn = false;
        if (turretLight != null) turretLight.setPosition(now < turretLightUntilMs ? LIGHT_ON : LIGHT_OFF);
        if (flyLight != null) flyLight.setPosition(flyLightOn ? LIGHT_ON : LIGHT_OFF);

        // ======================= ADDED: auto aim + clencher on the Driver Station =======================
        telemetry.addLine(locked ? "TURRET LOCKED" : "TURRET UNLOCKED");
        if (activeGoal < 0) {
            int[] cells = colorCells();
            telemetry.addData("Aim", "looking for the %s hive (tags %d-%d or %d-%d)", isRedAlliance ? "RED" : "BLUE",
                    CELL_BASES[cells[0]], CELL_BASES[cells[0]] + 3, CELL_BASES[cells[1]], CELL_BASES[cells[1]] + 3);
        } else {
            telemetry.addData("Aim", "%s by %s, turret %.0f deg -> %.0f deg%s", clusterName(CELL_BASES[activeGoal]),
                    aimer.source(now), turretDeg, tracking ? aimer.desiredDeg(now) : turretDeg, parked ? " (blind spot)" : "");
        }
        telemetry.addData("Camera sees", "%s%s", tagsInView, targetVisible ? String.format(Locale.US, ", tx %+.1f deg", target.tx) : "");
        telemetry.addData("Clencher", trans_open ? "DEPLOYED" : "RETRACTED");
        telemetry.addData("Intake", ltReverse ? "BACKWARDS" : ((in_run || ltIntakeOn || feedActive) ? "ON" : "off"));
        telemetry.update();

        if(gamepad1.y && !prevY){
            toggleLetOut();
            lastClencherSendMs = now;   // ADDED
        }
        prevY = gamepad1.y;
        // ADDED: re-send the clencher position every 0.1 s, so it holds even if a command gets lost
        if (now - lastClencherSendMs > 100) {
            transfer1.setPosition(trans_open ? clencherDeployed : clencherRetracted);
            lastClencherSendMs = now;
        }

        // ADDED: RT/LT are the shot and the intake now, so the hood stays where it is
        hood1.setPower(0);
        hood2.setPower(0);

        // Turret: D-pad left/right moves it by hand (as before); otherwise the AUTO AIM points it (ADDED)
        boolean turretManual = gamepad1.dpad_right || gamepad1.dpad_left;
        if (turretManual) manualHold = true;
        else if (manualHold && (targetVisible || tracking)) manualHold = false;   // aim takes over again
        double power = 0, tps = 0;
        boolean useVelocity = false;
        if(gamepad1.dpad_right) power = 0.5;
        else if(gamepad1.dpad_left) power = -0.5;
        else if (manualHold) power = 0;                              // stay where you left it until the hive is seen
        else if (tracking) { tps = aimer.update(now) * TURRET_TICKS_PER_DEGREE * TURRET_DIR; useVelocity = true; }
        else { tps = searchSpeedDegPerSec(turretDeg) * TURRET_TICKS_PER_DEGREE * TURRET_DIR; useVelocity = true; }
        if (turretManual || manualHold || tracking) searching = false;
        if (useVelocity && TurretTuning.maxAccelDegPerSec2 > 0) {   // optional smooth speed-up (TurretTuning)
            tps = limitSpeedUp(tps, lastTurretCmdTps, TurretTuning.maxAccelDegPerSec2 * TURRET_TICKS_PER_DEGREE * dt);
        }
        // Slow down near the limits, and never go past them (protects the wiring), by hand or by aim
        double degToMax = Math.max(0, (maxticks - ticks) / TURRET_TICKS_PER_DEGREE);
        double degToMin = Math.max(0, (ticks - minticks) / TURRET_TICKS_PER_DEGREE);
        if (useVelocity) tps = Math.max(-(TURRET_LIMIT_SLOWDOWN * degToMin + 20) * TURRET_TICKS_PER_DEGREE,
                Math.min((TURRET_LIMIT_SLOWDOWN * degToMax + 20) * TURRET_TICKS_PER_DEGREE, tps));
        double direction = useVelocity ? tps : power;
        maxed = (ticks >= maxticks && direction > 0) || (ticks <= minticks && direction < 0);
        if (maxed) { turret_aim.setPower(0); lastTurretCmdTps = 0; }
        else if (useVelocity) { turret_aim.setVelocity(tps); lastTurretCmdTps = tps; }
        else { turret_aim.setPower(power); lastTurretCmdTps = turret_aim.getVelocity(); }
    }

    private static double calculate_fly_speed(float flylen, double x, double y, double ang, Telemetry telemetry){
        try {
            double numerator = 9.8 * Math.pow(x, 2.0);
            double denominator = 2.0 * Math.pow(Math.cos(Math.toRadians(ang)), 2) * (x * Math.tan(Math.toRadians(ang)) - y);
            return ((1 / flylen) * Math.pow((numerator / denominator), 0.5));
        }
        catch (Exception e){
            telemetry.addLine("Error: Cannot shoot from here, you are likely too far away or smthn idk");
            return -1;
        }
    }

    private static double[] angleFromRPS(
            double flywheelRPS,
            double flywheelDiameterMeters,
            double gripConstant,
            double horizontalDistanceMeters, //x
            double heightDifferenceMeters //y
    ) {
        final double g = 9.81;

        double ballVelocity =
                gripConstant * Math.PI * flywheelDiameterMeters * flywheelRPS;

        double vSquared = ballVelocity * ballVelocity;

        // Quadratic in tan(theta):
        // A*tan(theta)^2 + B*tan(theta) + C = 0
        double A = (g * horizontalDistanceMeters * horizontalDistanceMeters)
                / (2.0 * vSquared);

        double B = -horizontalDistanceMeters;
        double C = A + heightDifferenceMeters;

        double discriminant = B * B - 4.0 * A * C;

        if (discriminant < 0) {
            return new double[0]; // RPS is too low for this target.
        }

        double lowTan = (-B - Math.sqrt(discriminant)) / (2.0 * A);
        double highTan = (-B + Math.sqrt(discriminant)) / (2.0 * A);

        double lowAngleDegrees = Math.toDegrees(Math.atan(lowTan));
        double highAngleDegrees = Math.toDegrees(Math.atan(highTan));

        return new double[] {lowAngleDegrees, highAngleDegrees};
    }
    // CHANGED: A just flips in_run; the intake and transfer power are set every loop in loop()
    // (A, LT and RT all share those motors). trans_power is still the transfer power for A.
    private void toggleIntake(double trans_power){
        aTransPower = trans_power;
        in_run = !in_run;
    }

    // CHANGED: uses the Clencher Tuner's positions: true = DEPLOYED, false = RETRACTED
    private void toggleLetOut(){
        trans_open = !trans_open;
        transfer1.setPosition(trans_open ? clencherDeployed : clencherRetracted);
    }

    // ======================= ADDED: auto aim helpers =======================

    /** What the camera sees of one hive (4 tags). */
    private static class Target {
        int count;
        double tx, area, frameAgeMs, distanceIn = Double.NaN;
    }

    /** Turret angle in degrees, + = right of where it pointed at INIT. */
    private double turretDegrees(int encoderTicks) {
        return (encoderTicks - initticks) / TURRET_TICKS_PER_DEGREE * TURRET_DIR;
    }

    private int[] colorCells() {
        return isRedAlliance ? RED_CELLS : BLUE_CELLS;
    }

    /** The hive of our color with the most tags in view (bigger area breaks ties), or -1. */
    private static int bestOf(Target[] seen, int[] allowed) {
        int best = -1;
        for (int i : allowed) {
            Target t = seen[i];
            if (t == null) continue;
            if (best < 0 || t.count > seen[best].count || (t.count == seen[best].count && t.area > seen[best].area)) best = i;
        }
        return best;
    }

    /** One entry per hive (CELL_BASES order), null where that hive isn't in view. tx = the hive's CENTER. */
    private Target[] findTargets(LLResult r, double nowMs) {
        Target[] out = new Target[CELL_BASES.length];
        tagsInView = "none";
        if (r == null || !r.isValid() || r.getStaleness() > MAX_FRAME_AGE_MS) return out;
        List<LLResultTypes.FiducialResult> fiducials = r.getFiducialResults();
        if (fiducials == null) return out;
        int[] member = new int[4];
        double[] tx = new double[4];
        double frameAgeMs = r.getStaleness() + r.getCaptureLatency() + r.getTargetingLatency();
        StringBuilder ids = new StringBuilder();
        for (LLResultTypes.FiducialResult f : fiducials) ids.append(ids.length() > 0 ? " " : "").append(f.getFiducialId());
        tagsInView = ids.length() > 0 ? ids.toString() : "none";
        int testCell = colorCells()[0];  // TurretTuning.testTagId (if set) counts as this hive
        for (int g = 0; g < CELL_BASES.length; g++) {
            Target t = new Target();
            double distSum = 0;
            int distCount = 0;
            for (LLResultTypes.FiducialResult f : fiducials) {
                int m = f.getFiducialId() - CELL_BASES[g];
                if (TurretTuning.testTagId >= 0 && f.getFiducialId() == TurretTuning.testTagId && g == testCell) m = 0;
                if (m < 0 || m > 3 || t.count >= 4) continue;
                member[t.count] = m;
                tx[t.count] = f.getTargetXDegrees();
                t.count++;
                t.area += f.getTargetArea();
                Pose3D p = f.getTargetPoseCameraSpace();
                if (p != null) {
                    Position pos = p.getPosition().toUnit(DistanceUnit.INCH);
                    double dist = Math.hypot(pos.x, pos.z);
                    if (dist > 1) { distSum += dist; distCount++; }
                }
            }
            if (t.count == 0) continue;
            t.tx = clusterFits[g].centerTx(nowMs, member, tx, t.count);
            if (distCount > 0) t.distanceIn = distSum / distCount;
            t.frameAgeMs = frameAgeMs;
            out[g] = t;
        }
        return out;
    }

    /** What the camera saw as one number: the same picture sent again gives the same number. */
    private static double pictureSignature(LLResult r) {
        if (r == null) return Double.NaN;
        List<LLResultTypes.FiducialResult> f = r.getFiducialResults();
        double s = 0;
        if (f != null) for (LLResultTypes.FiducialResult x : f) s += x.getFiducialId() + x.getTargetXDegrees() * 1.0001 + x.getTargetYDegrees() * 0.37;
        return s;
    }

    private static String clusterName(int base) {
        switch (base) {
            case RED_SCORING_BASE:   return "Red Hive (table side)";
            case RED_AUDIENCE_BASE:  return "Red Hive (audience side)";
            case BLUE_AUDIENCE_BASE: return "Blue Hive (audience side)";
            case BLUE_SCORING_BASE:  return "Blue Hive (table side)";
            default:                 return "Tags " + base + "-" + (base + 3);
        }
    }

    /**
     * Turret speed while looking for the hive (deg/s, + = right): slowly, around where a hive of our color
     * was last seen (corrected for how the robot turned since), a little wider each pass; never seen = end to end.
     */
    private double searchSpeedDegPerSec(double turretDeg) {
        double lo = Math.min(turretDegrees(minticks), turretDegrees(maxticks)) + 8;
        double hi = Math.max(turretDegrees(minticks), turretDegrees(maxticks)) - 8;
        double center = activeGoal >= 0 ? aimers[activeGoal].lastSeenBearingDeg() : Double.NaN;
        for (int g : colorCells()) if (Double.isNaN(center)) center = aimers[g].lastSeenBearingDeg();
        if (!searching) {
            searching = true;
            searchAmpDeg = 20;
            searchSign = 1;
        }
        double goal;
        if (Double.isNaN(center)) {
            if (turretDeg >= hi - 1) sweepDir = -1;
            else if (turretDeg <= lo + 1) sweepDir = 1;
            goal = sweepDir > 0 ? hi : lo;
        } else {
            center = reachableDeg(center, turretDeg, lo, hi);
            goal = Math.max(lo, Math.min(hi, center + searchSign * searchAmpDeg));
            if (Math.abs(goal - turretDeg) < 2) {
                searchSign = -searchSign;
                if (searchSign > 0) searchAmpDeg = Math.min(searchAmpDeg + 30, 360);
            }
        }
        double speed = 4.0 * (goal - turretDeg);
        return Math.max(-SEARCH_SWEEP_DEG_PER_SEC, Math.min(SEARCH_SWEEP_DEG_PER_SEC, speed));
    }

    /** The version of angle (+-360) inside [lo, hi] closest to current; in the blind spot, the nearer end. */
    private static double reachableDeg(double angle, double current, double lo, double hi) {
        double best = Double.NaN;
        for (int k = -2; k <= 2; k++) {
            double a = angle + 360.0 * k;
            if (a >= lo && a <= hi && (Double.isNaN(best) || Math.abs(a - current) < Math.abs(best - current))) best = a;
        }
        if (!Double.isNaN(best)) return best;
        double toHi = Math.abs(TurretAimer.wrap180(angle - hi)), toLo = Math.abs(TurretAimer.wrap180(angle - lo));
        return toHi <= toLo ? hi : lo;
    }

    /** Limit how fast a speed command grows; it may always shrink (or stop) at once. */
    static double limitSpeedUp(double cmd, double last, double maxStep) {
        if (last == 0 || Math.signum(cmd) == Math.signum(last)) {
            if (Math.abs(cmd) <= Math.abs(last)) return cmd;
            return last + Math.signum(cmd) * Math.min(Math.abs(cmd) - Math.abs(last), maxStep);
        }
        return Math.signum(cmd) * Math.min(Math.abs(cmd), maxStep);
    }

    /** Send TurretTuning's turret motor speed-loop gains to the hub (only when they changed). */
    private void applyTurretMotorPidf() {
        if (TurretTuning.motorP == appliedMotorP && TurretTuning.motorI == appliedMotorI
                && TurretTuning.motorD == appliedMotorD && TurretTuning.motorF == appliedMotorF) return;
        turret_aim.setVelocityPIDFCoefficients(TurretTuning.motorP, TurretTuning.motorI, TurretTuning.motorD, TurretTuning.motorF);
        appliedMotorP = TurretTuning.motorP;
        appliedMotorI = TurretTuning.motorI;
        appliedMotorD = TurretTuning.motorD;
        appliedMotorF = TurretTuning.motorF;
    }
}
