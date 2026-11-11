package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class LimelightCam {
    private final Limelight3A limelight;
    private final IMU imu;
    public LimelightCam(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0); //The settings/configs of the limelight, make it so you have pre-determined settings on the limelight
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    public void start(){
        limelight.start();
    }

    public LLResult getVars(Telemetry telemetry){
        YawPitchRollAngles orientation  = imu.getRobotYawPitchRollAngles();
        //TODO: maybe add limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult= limelight.getLatestResult();
/*        telemetry.addData("null?", llResult==(null));
        telemetry.addData("Valid?", llResult.isValid());
        telemetry.addData("pipeline:", llResult.getPipelineIndex());*/
        if(llResult != null){
            try {
                telemetry.addData("AllianceID", llResult.getFiducialResults().get(0).getFiducialId());
            }
            catch (Exception e){
                telemetry.addData("Error", e);
            }
            try {
                telemetry.addData("CodeID", llResult.getFiducialResults().get(1).getFiducialId());
            }
            catch (Exception e){
                telemetry.addData("Error:", "No second april tag in view");
            }

        }
        return llResult;
    }
}
