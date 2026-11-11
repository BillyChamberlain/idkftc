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

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

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

    private boolean trans_open;

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
        minticks = ticks - 770;
        maxticks = ticks + 580;


        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odcpu");

        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));


    }
    public void loop(){
        pinpoint.update();
        telemetry.addData("maxed?", maxed);
        ticks = turret_aim.getCurrentPosition();
        telemetry.addData("ticks", ticks);
        results = ll.getVars(telemetry);
        boolean tagFound = false;

// 1. Properly search for Tag 36 without overwriting the result
        Pose3D botpose = results.getBotpose();
        if(botpose !=null && results.isValid()) {

            double fieldX_meters = botpose.getPosition().x;
            double fieldY_meters = botpose.getPosition().y;
            double robotYaw_deg = botpose.getOrientation().getYaw();

            // C. HARD OVERRIDE: Inject Limelight absolute coordinates into the Pinpoint
            pinpoint.setPosition(new Pose2D(
                    DistanceUnit.METER,
                    fieldX_meters,
                    fieldY_meters,
                    AngleUnit.DEGREES,
                    robotYaw_deg
            ));
        }
        Pose2D currentPose = pinpoint.getPosition();

        telemetry.addData("X Position (in)", currentPose.getX(DistanceUnit.INCH));
        telemetry.addData("Y Position (in)", currentPose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)", currentPose.getHeading(AngleUnit.DEGREES));

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
        if(gamepad1.a && !prevA){
            toggleIntake(1);
        }
        prevA = gamepad1.a;
        double vertical = -gamepad1.left_stick_y;
        double horizontal = gamepad1.left_stick_x;
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
        telemetry.update();

        if(gamepad1.y && !prevY){
            toggleLetOut();
        }
        prevY = gamepad1.y;

        if(gamepad1.right_trigger_pressed){
            hood1.setPower(1);
            hood2.setPower(1);
        }
        else if(gamepad1.left_trigger_pressed){
            hood1.setPower(-1);
            hood2.setPower(-1);
        }
        else {
            hood1.setPower(0);
            hood2.setPower(0);
        }
        if(gamepad1.dpad_right) turret_aim.setPower(0.5);
        else if(gamepad1.dpad_left) turret_aim.setPower(-0.5);
        else turret_aim.setPower(0);
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
    private void toggleIntake(double trans_power){
        if(!in_run) {
            intake.setPower(-1.0);
            trans.setPower(-1 * trans_power);
        }
        else{
            intake.setPower(0);
            trans.setPower(0);
        }
        in_run = !in_run;
    }

    private void toggleLetOut(){
        if(trans_open) {
            transfer1.setPosition(0.3);
        }
        else{
            transfer1.setPosition(0.0);
        }
        trans_open = !trans_open;
    }
}
