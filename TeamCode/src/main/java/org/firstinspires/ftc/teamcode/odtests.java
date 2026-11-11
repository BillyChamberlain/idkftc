package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.util.HashSet;
import java.util.List;

public class odtests {
    private DcMotorEx axialEncoder, lateralEncoder;
    private IMU imu;
    private Limelight3A limelight;

    // Tracker coordinates (Global Field Position)
    private double robotX = 0; // Inches
    private double robotY = 0; // Inches
    private double robotHeading = 0; // Radians

    private int lastAxialTicks = 0;
    private int lastLateralTicks = 0;

    // goBILDA 4-bar/direct pod tracking constant
    // Formula: (Wheel Diameter [1.89055"] * Math.PI) / TicksPerRevolution [2000]
    private static final double INCHES_PER_TICK = (1.89055 * Math.PI) / 2000.0;

    // HashSet containing your 8 specific allowed AprilTag IDs
    private final HashSet<Integer> allowedTagIds = new HashSet<>();

    public odtests(HardwareMap hardwareMap) {
        // Initialize Encoder Ports & IMU
        axialEncoder = hardwareMap.get(DcMotorEx.class, "RB");
        lateralEncoder = hardwareMap.get(DcMotorEx.class, "RF");
        imu = hardwareMap.get(IMU.class, "imu");

        lastAxialTicks = axialEncoder.getCurrentPosition();
        lastLateralTicks = lateralEncoder.getCurrentPosition();

        // Initialize Limelight 3A via the FTC SDK
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Set to your AprilTag pipeline index
        limelight.start();

        // Populate your 8 target AprilTag IDs (Change these to match your field setup)
        for (int i = 30; i<=37; i++){
            allowedTagIds.add(i);
        }
    }

    public void update() {
        // --- STEP 1: READ ODOMETRY PODS EVERY LOOP CYCLE ---
        int currentAxial = axialEncoder.getCurrentPosition();
        int currentLateral = lateralEncoder.getCurrentPosition();

        int deltaAxialTicks = currentAxial - lastAxialTicks;
        int deltaLateralTicks = currentLateral - lastLateralTicks;

        lastAxialTicks = currentAxial;
        lastLateralTicks = currentLateral;

        double deltaAxialInches = deltaAxialTicks * INCHES_PER_TICK;
        double deltaLateralInches = deltaLateralTicks * INCHES_PER_TICK;

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        robotHeading = angles.getYaw(AngleUnit.RADIANS);

        // Track continuous motion relative to the field via dead reckoning
        double deltaX = deltaAxialInches * Math.cos(robotHeading) - deltaLateralInches * Math.sin(robotHeading);
        double deltaY = deltaAxialInches * Math.sin(robotHeading) + deltaLateralInches * Math.cos(robotHeading);

        robotX += deltaX;
        robotY += deltaY;

        // --- STEP 2: CHECK FOR LIMELIGHT ABSOLUTE SNAP ---
        // Feed your IMU heading context down to the Limelight hardware for MegaTag2 stability
        limelight.updateRobotOrientation(Math.toDegrees(robotHeading));

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            // Loop through all visible AprilTags in the current frame
            for (LLResultTypes.FiducialResult tag : fiducials) {

                // If any of the visible tags match your 8 allowed IDs, perform a full overwrite
                if (allowedTagIds.contains(tag.getFiducialId())) {

                    // Fetch the MegaTag2 robot pose relative to the field coordinate system
                    Pose3D botpose = result.getBotpose_MT2();

                    if (botpose != null) {
                        // The FTC SDK outputs Pose3D metrics in millimeters. Convert mm to inches (divide by 25.4)
                        robotX = botpose.getPosition().x / 25.4;
                        robotY = botpose.getPosition().y / 25.4;

                        // Break out early so we update immediately off the first valid approved tag we see
                        break;
                    }
                }
            }
        }
    }

    // Getters to feed your pathing / motor commands
    public double getX() { return robotX; }
    public double getY() { return robotY; }
    public double getHeading() { return robotHeading; }

    // Optional: Call this in your autonomous init loop to stop the camera stream when done
    public void stopCamera() {
        limelight.stop();
    }
}
