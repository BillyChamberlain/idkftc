package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Iterator;
import java.util.Locale;

/**
 * MOTOR TEST: run each motor on the EXPANSION HUB by itself, to check it works and spins the right way.
 *
 *   A = port 0     B = port 1     X = port 2     Y = port 3     (HOLD the button: the motor runs)
 *   LB held too    = run it BACKWARDS
 *   D-pad up/down  = test power +/- 10% (starts at 50%)
 *
 * The Driver Station shows, for each port: the motor's name, power, encoder position, speed and current.
 * The motors are found by PORT on the Expansion Hub (whatever they're named in the configuration).
 *
 * The turret motor ("aim") has wires: it runs at most 30% here, and if the Limits Tuner saved limits it
 * stops at them. Point the turret FORWARD before INIT so its limits line up.
 */
@TeleOp(name = "Motor Test", group = "Tuning")
public class MotorTest extends OpMode {

    private static final double TURRET_MAX_POWER = 0.3;
    private static final int TURRET_LIMIT_MARGIN = 40;   // ticks before a saved turret limit

    private final DcMotorEx[] motors = new DcMotorEx[4];   // index = Expansion Hub port
    private final String[] names = new String[4];
    private final double[] amps = new double[4];
    private int[] turretLimits;     // Limits Tuner's saved limits (ticks from forward), or null
    private int turretStart;        // turret encoder at INIT (= forward)
    private double testPower = 0.5;
    private boolean prevUp, prevDown;
    private String hubText = "";
    private long lastAmpsMs = 0;

    @Override
    public void init() {
        // Which hub is the Expansion Hub? (the Control Hub is the "parent")
        int expansionAddress = -1;
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            if (!hub.isParent()) expansionAddress = hub.getModuleAddress();
        }
        hubText = expansionAddress < 0 ? "NO EXPANSION HUB FOUND" : "Expansion Hub (address " + expansionAddress + ")";

        // Every motor on that hub, by port. (A motor's connection info reads like "...; module 1; port 3".)
        for (DcMotorEx m : hardwareMap.getAll(DcMotorEx.class)) {
            String info = m.getConnectionInfo();
            int port = m.getPortNumber();
            if (expansionAddress < 0 || info == null || !info.contains("module " + expansionAddress + ";")) continue;
            if (port < 0 || port > 3) continue;
            motors[port] = m;
            Iterator<String> it = hardwareMap.getNamesOf(m).iterator();
            names[port] = it.hasNext() ? it.next() : "?";
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // plain power (the encoder still counts)
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setPower(0);
            if ("aim".equals(names[port])) turretStart = m.getCurrentPosition();
        }
        turretLimits = LimitsTuner.loadLimits();
        telemetry.addLine("MOTOR TEST: " + hubText);
        for (int p = 0; p < 4; p++) telemetry.addLine("  port " + p + ": " + (motors[p] == null ? "(no motor)" : names[p]));
        telemetry.addLine("Turret (aim): point it FORWARD now. Press PLAY, then HOLD A / B / X / Y.");
    }

    @Override
    public void loop() {
        // Test power: D-pad up/down
        if (gamepad1.dpad_up && !prevUp) testPower = Math.min(1.0, testPower + 0.1);
        if (gamepad1.dpad_down && !prevDown) testPower = Math.max(0.1, testPower - 0.1);
        prevUp = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;
        double dir = gamepad1.left_bumper ? -1 : 1;
        boolean[] held = {gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y};

        // Read currents 10 times a second (each read is a separate, slower command)
        long nowMs = System.currentTimeMillis();
        boolean readAmps = nowMs - lastAmpsMs > 100;
        if (readAmps) lastAmpsMs = nowMs;

        telemetry.addLine(String.format(Locale.US, "MOTOR TEST  %s  ·  test power %.0f%% %s", hubText, testPower * 100,
                gamepad1.left_bumper ? "BACKWARDS (LB)" : "forward"));
        String[] buttons = {"A", "B", "X", "Y"};
        for (int p = 0; p < 4; p++) {
            DcMotorEx m = motors[p];
            if (m == null) {
                telemetry.addLine(String.format(Locale.US, "%s  port %d: (no motor configured)", buttons[p], p));
                continue;
            }
            double power = held[p] ? dir * testPower : 0;
            String note = "";
            if ("aim".equals(names[p])) {   // the turret: gentle, and never past its saved limits
                power = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, power));
                int t = m.getCurrentPosition() - turretStart;
                if (turretLimits != null && ((power > 0 && t >= turretLimits[1] - TURRET_LIMIT_MARGIN)
                        || (power < 0 && t <= turretLimits[0] + TURRET_LIMIT_MARGIN))) {
                    power = 0;
                    note = "  AT TURRET LIMIT";
                }
            }
            m.setPower(power);
            if (readAmps) amps[p] = power != 0 ? m.getCurrent(CurrentUnit.AMPS) : 0;
            telemetry.addLine(String.format(Locale.US, "%s  port %d %-9s %s  power %+.2f  enc %7d  %6.0f ticks/s  %4.1f A%s",
                    buttons[p], p, names[p], power != 0 ? "RUNNING" : "  off  ", power, m.getCurrentPosition(),
                    m.getVelocity(), amps[p], note));
        }
        telemetry.addLine();
        telemetry.addLine("HOLD A/B/X/Y = run port 0/1/2/3  |  + LB = backwards  |  D-pad up/down = power");
        telemetry.update();
    }

    @Override
    public void stop() {
        for (DcMotorEx m : motors) if (m != null) m.setPower(0);
    }
}
