package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.List;
import java.util.Locale;

/**
 * Transfer Test: load balls and have the transfer stop by itself when it is full.
 *
 * The hub can't measure a motor's voltage, but it CAN measure the current the motor
 * draws, and that goes up as balls pile up against the closed clencher. So we watch the
 * transfer motor's current and stop when it passes the "full" level for the ball count
 * you picked. The levels are LEARNED here: load that many balls, press LB, done.
 *
 * Controls (gamepad1):
 *   X          pick 1 / 2 / 3 / 4 balls (rumbles that many times)
 *   RB         AUTO LOAD: run intake + transfer, stop when the current says it's full
 *   A          run intake + transfer by hand (same as A in the main TeleOp; no auto stop here)
 *   LT (hold)  run both backwards (clear a jam)
 *   LB         LEARN: with this many balls loaded and the transfer pushing against the
 *              closed clencher, save the current reading as "full" for this ball count
 *   Y          clencher: retracted <-> deployed (same as the main TeleOp)
 *   B          flywheel on / off (for outtaking)
 *   D-pad up/down   flywheel speed +50 / -50 ticks per second
 *   BACK       forget the learned levels
 * The motors cut out by themselves if the transfer stalls (current very high for 1.5 s).
 */
@TeleOp(name = "Transfer Test", group = "Tuning")
public class TransferTest extends OpMode {

    static final File FULL_FILE = new File(AppUtil.FIRST_FOLDER, "transfer_full_amps.csv");

    // Same directions as the main TeleOp: negative power = balls go in and up
    private static final double LOAD_POWER = -1.0;
    /** Until a level is learned for a ball count, stop at this current (a guess: learn it with LB). */
    private static final double DEFAULT_FULL_AMPS = 1.5;
    /** Motor start-up surge: ignore the current this long after starting. */
    private static final double STARTUP_IGNORE_MS = 600;
    /** The current must stay above the level this long before we call it full. */
    private static final double HOLD_MS = 200;
    /** Stop the intake too when full, so it doesn't push more balls into a full transfer. */
    private static final boolean STOP_INTAKE_WHEN_FULL = true;
    /** Learned level = the reading at LB times this, so it triggers just below what we measured. */
    private static final double LEARN_MARGIN = 0.92;
    /** Stall protection: current above this for STALL_MS cuts both motors. */
    private static final double STALL_AMPS = 6.0;
    private static final double STALL_MS = 1500;
    private static final double DEFAULT_FLYWHEEL_TPS = 1200;

    private enum Mode { OFF, MANUAL, AUTO }

    private DcMotor intake;
    private DcMotorEx trans, shoot;
    private Servo clencher;
    private VoltageSensor battery;
    private final FlywheelController flywheel = new FlywheelController();
    private final ElapsedTime clock = new ElapsedTime();

    private final double[] fullAmps = new double[4];
    private final boolean[] learned = new boolean[4];
    private int count = 1;
    private Mode mode = Mode.OFF;
    private double modeStartMs, aboveSinceMs = Double.NaN, stallSinceMs = Double.NaN;
    private double lastMs = Double.NaN;

    private double amps, avgAmps, peakAmps;
    private final double[] recent = new double[25];  // last ~0.5 s of readings, for LB
    private int recentCount = 0, recentNext = 0;

    private boolean flyOn, clencherOpen;
    private double flyTps = DEFAULT_FLYWHEEL_TPS;
    private double clencherShut, clencherDeployed, lastClencherSendMs = Double.NEGATIVE_INFINITY;
    private double volts = 12.0, lastVoltsMs = Double.NEGATIVE_INFINITY;

    private boolean prevX, prevA, prevB, prevY, prevLB, prevRB, prevUp, prevDown, prevBack;
    private String message = "";

    // ---------------- Learned levels file ----------------

    private void loadLevels() {
        for (int i = 0; i < 4; i++) { fullAmps[i] = DEFAULT_FULL_AMPS; learned[i] = false; }
        if (!FULL_FILE.exists()) return;
        try (BufferedReader r = new BufferedReader(new FileReader(FULL_FILE))) {
            String line;
            while ((line = r.readLine()) != null) {
                String[] p = line.trim().split(",");
                if (p.length < 2 || !Character.isDigit(p[0].trim().isEmpty() ? 'x' : p[0].trim().charAt(0))) continue;
                int n = Integer.parseInt(p[0].trim());
                double a = Double.parseDouble(p[1].trim());
                if (n >= 1 && n <= 4 && a > 0.05 && a < 12) { fullAmps[n - 1] = a; learned[n - 1] = true; }
            }
        } catch (Exception ignored) { }
    }

    private boolean saveLevels() {
        try {
            File dir = FULL_FILE.getParentFile();
            if (dir != null && !dir.exists()) dir.mkdirs();
            try (FileWriter w = new FileWriter(FULL_FILE)) {
                w.write("balls,full_amps\n");
                for (int i = 0; i < 4; i++) {
                    if (learned[i]) w.write(String.format(Locale.US, "%d,%.3f%n", i + 1, fullAmps[i]));
                }
            }
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    // ---------------- OpMode ----------------

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        trans = hardwareMap.get(DcMotorEx.class, "transfer");
        shoot = hardwareMap.get(DcMotorEx.class, "shoot");
        clencher = hardwareMap.get(Servo.class, "t1");
        shoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // our own speed controller, like the main TeleOp
        shoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        double[] c = ClencherTuner.positionsInUse();  // same positions as the main TeleOp
        clencherShut = c[0];
        clencherDeployed = c[1];

        List<VoltageSensor> sensors = hardwareMap.getAll(VoltageSensor.class);
        battery = sensors.isEmpty() ? null : sensors.get(0);
        loadLevels();

        telemetry.addLine("Transfer Test: press PLAY. The clencher goes to RETRACTED.");
        telemetry.addLine("Pick the ball count with X, load with RB (auto) or A (manual).");
    }

    @Override
    public void start() {
        clock.reset();
        clencherOpen = false;
        clencher.setPosition(clencherShut);
        lastClencherSendMs = 0;
        prevX = gamepad1.x; prevA = gamepad1.a; prevB = gamepad1.b; prevY = gamepad1.y;
        prevLB = gamepad1.left_bumper; prevRB = gamepad1.right_bumper;
        prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down; prevBack = gamepad1.back;
    }

    @Override
    public void loop() {
        double now = clock.milliseconds();
        double dt = Double.isNaN(lastMs) ? 0.02 : (now - lastMs) / 1000.0;
        lastMs = now;
        if (battery != null && now - lastVoltsMs > 500) { volts = battery.getVoltage(); lastVoltsMs = now; }

        // ---- Current the transfer motor is drawing (goes up as balls pile up) ----
        amps = trans.getCurrent(CurrentUnit.AMPS);
        avgAmps += 0.3 * (amps - avgAmps);
        recent[recentNext] = amps;
        recentNext = (recentNext + 1) % recent.length;
        if (recentCount < recent.length) recentCount++;
        boolean running = mode != Mode.OFF;
        if (!running) peakAmps = 0;
        else if (now - modeStartMs > STARTUP_IGNORE_MS) peakAmps = Math.max(peakAmps, avgAmps);

        // ---- Buttons ----
        if (gamepad1.x && !prevX) {
            count = count % 4 + 1;
            gamepad1.rumbleBlips(count);
            message = "";
        }
        if (gamepad1.right_bumper && !prevRB) startOrStop(Mode.AUTO, now);
        if (gamepad1.a && !prevA) startOrStop(Mode.MANUAL, now);
        if (gamepad1.left_bumper && !prevLB) learn();
        if (gamepad1.y && !prevY) { clencherOpen = !clencherOpen; sendClencher(now); }
        if (gamepad1.b && !prevB) { flyOn = !flyOn; flywheel.reset(); }
        if (gamepad1.dpad_up && !prevUp) flyTps = Math.min(2700, flyTps + 50);
        if (gamepad1.dpad_down && !prevDown) flyTps = Math.max(0, flyTps - 50);
        if (gamepad1.back && !prevBack) {
            for (int i = 0; i < 4; i++) { fullAmps[i] = DEFAULT_FULL_AMPS; learned[i] = false; }
            if (FULL_FILE.exists()) FULL_FILE.delete();
            message = "Learned levels deleted.";
        }
        prevX = gamepad1.x; prevA = gamepad1.a; prevB = gamepad1.b; prevY = gamepad1.y;
        prevLB = gamepad1.left_bumper; prevRB = gamepad1.right_bumper;
        prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down; prevBack = gamepad1.back;

        // ---- Auto stop: full when the current stays above this count's level ----
        if (mode == Mode.AUTO && now - modeStartMs > STARTUP_IGNORE_MS) {
            if (avgAmps >= fullAmps[count - 1]) {
                if (Double.isNaN(aboveSinceMs)) aboveSinceMs = now;
                if (now - aboveSinceMs >= HOLD_MS) {
                    message = String.format(Locale.US, "FULL for %d ball%s: %.2f A (level %.2f A)",
                            count, count == 1 ? "" : "s", avgAmps, fullAmps[count - 1]);
                    stopMotors(STOP_INTAKE_WHEN_FULL);
                    gamepad1.rumble(400);
                }
            } else {
                aboveSinceMs = Double.NaN;
            }
        }

        // ---- Stall protection ----
        if (mode != Mode.OFF && now - modeStartMs > STARTUP_IGNORE_MS && avgAmps >= STALL_AMPS) {
            if (Double.isNaN(stallSinceMs)) stallSinceMs = now;
            if (now - stallSinceMs >= STALL_MS) {
                message = String.format(Locale.US, "STALL PROTECTION: %.1f A for 1.5 s. Stopped. Check for a jam.", avgAmps);
                stopMotors(true);
                gamepad1.rumbleBlips(3);
            }
        } else {
            stallSinceMs = Double.NaN;
        }

        // ---- Motors ----
        boolean reverse = gamepad1.left_trigger > 0.3;
        double p = reverse ? -LOAD_POWER : (mode != Mode.OFF ? LOAD_POWER : 0);
        trans.setPower(p);
        intake.setPower(reverse ? -LOAD_POWER : (intakeOn ? LOAD_POWER : 0));

        // ---- Flywheel (for outtaking) ----
        double flyVel = shoot.getVelocity();
        if (flyOn) shoot.setPower(flywheel.update(flyTps, flyVel, volts, dt));
        else shoot.setPower(0);

        // ---- Clencher: re-sent every 100 ms, like Clencher Tuner ----
        if (now - lastClencherSendMs > 100) sendClencher(now);

        // ---- Screen ----
        telemetry.addData("Balls selected", "%d   (X to change)", count);
        telemetry.addData("Transfer", mode == Mode.AUTO ? "AUTO LOAD, running" : mode == Mode.MANUAL ? "MANUAL, running"
                : "stopped" + (reverse ? " (reversing)" : ""));
        telemetry.addData("Motor current", "%.2f A  %s  (peak %.2f A)", avgAmps, bar(avgAmps, 6.0, 24), peakAmps);
        telemetry.addData("Full level for " + count, "%.2f A  (%s)", fullAmps[count - 1], learned[count - 1] ? "learned" : "guess: learn it");
        StringBuilder all = new StringBuilder();
        for (int i = 0; i < 4; i++) {
            all.append(i + 1).append(": ").append(String.format(Locale.US, "%.2f", fullAmps[i])).append(learned[i] ? " L" : " -").append(i < 3 ? "   " : "");
        }
        telemetry.addData("All levels (L = learned)", all.toString());
        telemetry.addData("Clencher", "%s  (servo %.2f)", clencherOpen ? "DEPLOYED" : "RETRACTED", clencher.getPosition());
        telemetry.addData("Flywheel", flyOn ? String.format(Locale.US, "ON  target %.0f  actual %.0f t/s", flyTps, flyVel)
                : String.format(Locale.US, "off  (target %.0f t/s)", flyTps));
        if (!message.isEmpty()) telemetry.addLine(message);
        telemetry.addLine();
        telemetry.addLine("X count | RB auto load | A manual | LT reverse | LB learn | Y clencher | B flywheel | D-pad speed");
        telemetry.update();
    }

    @Override
    public void stop() {
        trans.setPower(0);
        intake.setPower(0);
        shoot.setPower(0);
    }

    // ---------------- Helpers ----------------

    private boolean intakeOn;  // can stay on after AUTO stops, if STOP_INTAKE_WHEN_FULL is false

    private void startOrStop(Mode wanted, double now) {
        if (mode == wanted) {
            stopMotors(true);
            message = "Stopped by hand.";
        } else {
            mode = wanted;
            modeStartMs = now;
            aboveSinceMs = Double.NaN;
            stallSinceMs = Double.NaN;
            peakAmps = 0;
            intakeOn = true;
            message = "";
        }
    }

    private void stopMotors(boolean alsoIntake) {
        mode = Mode.OFF;
        aboveSinceMs = Double.NaN;
        if (alsoIntake) intakeOn = false;
        trans.setPower(0);
    }

    /** LB: save the reading from the last half second as the "full" level for this ball count. */
    private void learn() {
        double sum = 0;
        for (int i = 0; i < recentCount; i++) sum += recent[i];
        double avg = recentCount > 0 ? sum / recentCount : 0;
        if (mode == Mode.OFF || avg < 0.2) {
            message = "NOT learned: run the transfer (A) with " + count + " ball" + (count == 1 ? "" : "s")
                    + " loaded and pushing the clencher, then press LB.";
            return;
        }
        fullAmps[count - 1] = avg * LEARN_MARGIN;
        learned[count - 1] = true;
        message = String.format(Locale.US, "LEARNED %d ball%s: reading %.2f A, stops at %.2f A. %s",
                count, count == 1 ? "" : "s", avg, fullAmps[count - 1], saveLevels() ? "Saved." : "(could not save)");
        gamepad1.rumbleBlips(2);
    }

    private void sendClencher(double now) {
        clencher.setPosition(clencherOpen ? clencherDeployed : clencherShut);
        lastClencherSendMs = now;
    }

    private static String bar(double value, double max, int width) {
        int filled = (int) Math.round(Math.max(0, Math.min(1, value / max)) * width);
        StringBuilder b = new StringBuilder("[");
        for (int i = 0; i < width; i++) b.append(i < filled ? '#' : '-');
        return b.append(']').toString();
    }
}
