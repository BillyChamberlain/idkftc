package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing (classic PIDF version) settings for THIS robot.
 *
 * Everything not set below keeps Pedro's default, which already drives a typical robot reasonably.
 * Tune the rest with the "Tuning" OpMode (Driver Station > TeleOp > "Pedro Pathing" group) and the
 * Panels dashboard in a browser at http://192.168.43.1:8001
 */
public class Constants {

    // Robot weight in kg (used for the curve-following force). 10.65 is Pedro's default:
    // weigh the robot (with battery) and put the real number here.
    // Zero-power accelerations (how fast it slows down with power cut, motors floating): forward is the
    // average of three tuner runs (-40.7, -35.9, -31.8). Lateral was not run: it is the forward value
    // scaled by Pedro's default lateral/forward ratio (59.78 / 41.28).
    // Translational PIDF: tuned by hand in the Translational Tuner (P kept at the default 0.1, D added).
    // Heading PIDF: tuned by hand in the Heading Tuner (P 1.3, up from the default 1.0; F stays 0.01).
    // Drive PIDF stays at Pedro's default.
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-36.16)
            .lateralZeroPowerAcceleration(-52.4)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.005, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.3, 0, 0, 0.01));

    // Motor names and directions are from mechanumtest, confirmed by Pedro's motor-direction test
    // (only the back right is reversed). xVelocity: average of three Forward Velocity tuner runs
    // (103.0, 101.5, 102.8). yVelocity: the one Lateral Velocity run read 56.6 but the robot curved
    // during it (which lowers the reading), and strafing felt like about 75% of forward speed in the
    // TeleOp, so it is set to 75% of xVelocity. Rerun the Lateral Velocity tuner if sideways moves look off.
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .xVelocity(102.43)
            .yVelocity(76.8)
            .leftFrontMotorName("LF")
            .leftRearMotorName("LB")
            .rightFrontMotorName("RF")
            .rightRearMotorName("RB")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    // goBILDA Pinpoint, measured with Pedro's Pinpoint tuner (see mechanumtest): the Y (strafe) pod is
    // reversed, the forward pod sits 1.15 in left of center and the strafe pod 2.94 in behind it.
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .hardwareMapName("odcpu")
            .distanceUnit(DistanceUnit.INCH)
            .forwardPodY(1.1514677212932918)
            .strafePodX(-2.941031869002215)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    /** Build the path follower: use this in an autonomous OpMode. */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
