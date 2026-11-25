package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

/**
 * Clencher Tuner: find the clencher's RETRACTED and DEPLOYED servo positions.
 *
 * A normal servo can't report its real angle, so this lets you nudge it and read
 * the number. Mark where the horn points, nudge it to each spot you want, and save:
 *   D-pad up/down   nudge 0.01 (hold to keep going)
 *   RB / LB         nudge 0.05
 *   A               save this spot as RETRACTED (start)
 *   B               save this spot as DEPLOYED (end)
 *   Y               jump between the two saved spots to test them
 * A and B write the positions to the Control Hub, and mechanumtest and Transfer Test
 * use them from their next INIT: no numbers to copy by hand.
 */
@TeleOp(name = "Clencher Tuner", group = "Tuning")
public class ClencherTuner extends OpMode {

    private Servo servo;
    static final File POSITIONS_FILE = new File(AppUtil.FIRST_FOLDER, "clencher_positions.txt");
    private final ElapsedTime clock = new ElapsedTime();
    private double pos, retracted, deployed;
    private String message = "";
    private boolean atDeployed;
    private boolean prevA, prevB, prevY, prevLB, prevRB;
    private double heldSinceMs = Double.NaN, lastStepMs;

    /** Saved {retracted, deployed} positions (0..1) from this OpMode, or null if none saved. */
    static double[] loadPositions() {
        if (!POSITIONS_FILE.exists()) return null;
        try (BufferedReader r = new BufferedReader(new FileReader(POSITIONS_FILE))) {
            String line = r.readLine();
            if (line == null) return null;
            String[] p = line.trim().split(",");
            if (p.length < 2) return null;
            double a = Double.parseDouble(p[0].trim()), b = Double.parseDouble(p[1].trim());
            if (a < 0 || a > 1 || b < 0 || b > 1 || Math.abs(a - b) < 0.02) return null;
            return new double[]{a, b};
        } catch (Exception e) {
            return null;
        }
    }

    /** What the OpModes actually use: saved positions if any, else the defaults in mechanumtest. */
    static double[] positionsInUse() {
        double[] saved = loadPositions();
        return saved != null ? saved : new double[]{mechanumtest.CLENCHER_SHUT_POS, mechanumtest.CLENCHER_OPEN_POS};
    }

    private boolean savePositions() {
        try {
            File dir = POSITIONS_FILE.getParentFile();
            if (dir != null && !dir.exists()) dir.mkdirs();
            try (FileWriter w = new FileWriter(POSITIONS_FILE)) {
                w.write(String.format(java.util.Locale.US, "%.3f,%.3f%n", retracted, deployed));
            }
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "t1");
        double[] p = positionsInUse();
        retracted = p[0];
        deployed = p[1];
        pos = retracted;
        telemetry.addLine("Clencher Tuner: press PLAY. The servo moves to the current RETRACTED spot ("
                + fmt(retracted) + ").");
        telemetry.addLine("t1 is on: " + servo.getConnectionInfo());
    }

    @Override
    public void start() {
        clock.reset();
        servo.setPosition(pos);
    }

    @Override
    public void loop() {
        double now = clock.milliseconds();

        // D-pad: one 0.01 step per press, then repeats every 80 ms after holding 0.4 s
        int dir = gamepad1.dpad_up ? 1 : (gamepad1.dpad_down ? -1 : 0);
        if (dir == 0) {
            heldSinceMs = Double.NaN;
        } else if (Double.isNaN(heldSinceMs)) {
            heldSinceMs = now;
            lastStepMs = now;
            pos += 0.01 * dir;
        } else if (now - heldSinceMs > 400 && now - lastStepMs > 80) {
            lastStepMs = now;
            pos += 0.01 * dir;
        }
        if (gamepad1.right_bumper && !prevRB) pos += 0.05;
        if (gamepad1.left_bumper && !prevLB) pos -= 0.05;

        if (gamepad1.a && !prevA) { retracted = pos; message = savePositions() ? "RETRACTED saved." : "Could not save the file."; gamepad1.rumble(200); }
        if (gamepad1.b && !prevB) { deployed = pos; message = savePositions() ? "DEPLOYED saved." : "Could not save the file."; gamepad1.rumble(200); }
        if (gamepad1.y && !prevY) {
            atDeployed = !atDeployed;
            pos = atDeployed ? deployed : retracted;
        }
        prevA = gamepad1.a; prevB = gamepad1.b; prevY = gamepad1.y;
        prevLB = gamepad1.left_bumper; prevRB = gamepad1.right_bumper;

        pos = Math.max(0, Math.min(1, Math.round(pos * 1000) / 1000.0));
        servo.setPosition(pos);

        double span = Math.abs(deployed - retracted);
        telemetry.addData("Servo position now", fmt(pos));
        telemetry.addData("Pulse", "%.0f us", 600 + pos * 1800);
        telemetry.addLine();
        telemetry.addData("RETRACTED (A saves)", fmt(retracted));
        telemetry.addData("DEPLOYED  (B saves)", fmt(deployed));
        telemetry.addData("Gap", "%.3f = about %.0f deg (goBILDA) / %.0f deg (REV Smart Servo)",
                span, span * 270, span * 243);
        telemetry.addData("Deployed is", deployed < retracted ? "LOWER number" : "HIGHER number");
        if (!message.isEmpty()) telemetry.addLine(message);
        telemetry.addLine();
        telemetry.addLine("D-pad up/down 0.01  |  RB/LB 0.05  |  Y test both spots");
        telemetry.addLine("A and B save to the robot: mechanumtest uses them from its next INIT.");
        telemetry.update();
    }

    private static String fmt(double v) { return String.format(java.util.Locale.US, "%.3f", v); }
}
