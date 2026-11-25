package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * TURRET CHECK (camera calibration), used by the Limits Tuner (RB starts it).
 *
 * The aim turns "degrees" into motor ticks with the turret's ticks per degree, and it assumes which way
 * the turret has to turn when the robot turns. If either is wrong, the aim overshoots, wobbles and
 * swings around, no matter how good the aim code is. This check measures both with the camera, so no
 * protractor or "exactly 90 degrees" guess is needed.
 *
 *  Step 1, TICKS PER DEGREE (automatic): park facing ANY AprilTag. The turret moves in small steps; at
 *    each stop it waits for the belt to settle, then averages the camera's tx of that tag. The camera
 *    measures angles exactly, so a straight-line fit of encoder ticks against tx gives the real ticks
 *    per degree. Going both ways cancels the belt's slack.
 *  Step 2, TURN DIRECTION (driver turns the robot slowly with the right stick, turret held still): when
 *    the robot turns LEFT (gyro heading goes up), a tag in front should move RIGHT in the picture (tx
 *    goes up). If it moves the other way, the turret/camera are mirrored compared with the gyro, and the
 *    main TeleOp has to mirror the gyro for the aim (it does that when this check saves "-1").
 *
 * How it copes with a real turret (each of these was tested in a simulator before going on the robot):
 *  - every stop is reached from the SAME side (it overshoots a little and comes back), so belt slack
 *    doesn't bend the result; what's left of the slack is fitted separately and shown
 *  - each stop is averaged over almost a second of pictures, so belt wobble averages out
 *  - the first steps start small and only grow once the tag has really moved (max 2x per stop)
 *  - if the encoder stops moving it pushes a bit harder; still stuck = a hard stop: it backs off
 *  - it stays inside the given tick range, and only refuses moves that would go further OUT
 *  - before step 2 it turns the turret back so the tag is in the middle of the picture
 *  - step 2 needs at least one back-and-forth so camera lag can't fake the answer
 *
 * Plain Java (no hardware), so it can be simulated on a laptop. The Limits Tuner feeds it the encoder,
 * the gyro and each NEW camera picture, and sends the turret power it asks for.
 */
public class TurretCheck {

    public enum Stage { IDLE, SCALE, HEADING, DONE, FAILED }

    // ---------------- Settings ----------------
    /** After each turret move, wait this long (belt wobble dies down), then average the camera for averageMs. */
    public double settleMs = 600;
    public double averageMs = 900;
    /** At least this many camera pictures per stop. */
    public int picturesPerStop = 6;
    /** Aim for about this many degrees of turret move per step. */
    public double stepDeg = 3.0;
    /** Turn around when the tag gets this far from the middle of the picture (degrees). */
    public double edgeDeg = 13.0;
    /** Most stops in step 1. */
    public int maxStops = 24;
    /** Turret power while stepping (it pushes a bit harder if the turret doesn't move). */
    public double maxPower = 0.25, minPower = 0.07;
    /** Step 2 needs this much robot turning in total (degrees) and this many pictures. */
    public double headingSpanDeg = 20.0;
    public int headingPictures = 20;
    /** Fail step 1 if the ticks-per-degree is less certain than this (percent). */
    public double maxSlopeErrPct = 2.5;
    /** Reach every stop from the ticks-up side (overshoot by oneSideOverTicks and come back): belt slack
     *  is then taken up the same way at every stop. */
    public boolean approachFromOneSide = true;
    public double oneSideOverTicks = 30;   // how far past a stop it goes before coming back
    private double pendingTarget = Double.NaN;

    public Stage stage = Stage.IDLE;
    public String status = "Press RB to start the camera check (park facing any AprilTag).";
    // ---------------- Results ----------------
    /** Measured turret ticks per degree, or NaN. */
    public double ticksPerDeg = Double.NaN;
    /** How far the stops are from the fitted line (degrees), how much belt slack it saw (degrees), and how
     *  sure the ticks-per-degree is (+- percent). */
    public double fitErrorDeg = Double.NaN;
    public double backlashDeg = Double.NaN;
    public double slopeErrPct = Double.NaN;
    /** true if more ticks x TURRET_DIR (turret "right") moves the tag LEFT in the picture, as the aim expects. */
    public boolean turretDirOk = true;
    /** tx change per degree of gyro heading change: about +1 normal, about -1 mirrored. NaN until measured. */
    public double headingRatio = Double.NaN;
    /** +1 normal, -1 mirrored, 0 not measured / unclear. */
    public int headingSign = 0;

    private final int turretDir;
    private double minTicks, maxTicks;
    private int moveDir = 1, lastMoveDir = 0;
    private int turns = 0;
    private double stepTicks = 20;
    private double targetTicks;
    private double moveStartMs, settleUntilMs, lastTagSeenMs, collectStartMs;
    private boolean moving, collecting, recentering;
    private double boost = 0, lastProgressTicks, lastProgressMs;
    private boolean blocked;
    private double txSum; private int txCount;
    private final List<double[]> stops = new ArrayList<>();     // {ticks, tx, dir}
    private final List<double[]> headingPts = new ArrayList<>(); // {heading, tx}
    private double fitA, fitSlope, fitH;

    public TurretCheck(int turretDir) { this.turretDir = turretDir >= 0 ? 1 : -1; }

    public void start(double nowMs, int ticks, double minTicks, double maxTicks) {
        this.minTicks = Math.min(minTicks, maxTicks);
        this.maxTicks = Math.max(minTicks, maxTicks);
        stops.clear(); headingPts.clear();
        ticksPerDeg = fitErrorDeg = headingRatio = backlashDeg = slopeErrPct = Double.NaN;
        headingSign = 0; turretDirOk = true; turns = 0; stepTicks = 20; moveDir = 1; lastMoveDir = 0;
        boost = 0; moving = false; recentering = false; blocked = false;
        prevHeading = Double.NaN; turnDir = 0; lastTurnDir = 0; reversals = 0;
        lastTagSeenMs = nowMs;
        stage = Stage.SCALE;
        beginCollect(nowMs);
        status = "Step 1: measuring ticks per degree. Keep the robot still.";
    }


    public double update(double nowMs, int ticks, double headingDeg, boolean newPicture, double tagTx) {
        boolean tagSeen = newPicture && !Double.isNaN(tagTx);
        if (tagSeen) lastTagSeenMs = nowMs;
        switch (stage) {
            case SCALE:
                if (nowMs - lastTagSeenMs > 2000) return fail("The camera lost the tag. Park facing an AprilTag (near the middle of the picture) and press RB again.");
                return scaleStep(nowMs, ticks, tagSeen, tagTx);
            case HEADING:
                // Note which way the robot is turning; step 2 needs a back-and-forth, and the fit has a
                // turn-direction term so camera lag (old picture, newer heading) cannot fake the answer
                if (!Double.isNaN(prevHeading) && nowMs > prevHeadingMs) {
                    double rate = (headingDeg - prevHeading) / (nowMs - prevHeadingMs) * 1000;
                    turnDir = Math.abs(rate) > 5 ? (int) Math.signum(rate) : 0;
                    if (turnDir != 0) {
                        if (lastTurnDir != 0 && turnDir != lastTurnDir) reversals++;
                        lastTurnDir = turnDir;
                    }
                }
                prevHeading = headingDeg; prevHeadingMs = nowMs;
                if (tagSeen) headingPts.add(new double[]{headingDeg, tagTx, needBackAndForth ? turnDir : 0});
                double span = span(headingPts, 0);
                int nPlus = 0, nMinus = 0;
                for (double[] q : headingPts) { if (q[2] > 0) nPlus++; if (q[2] < 0) nMinus++; }
                boolean bothWays = !needBackAndForth || (reversals >= 1 && nPlus >= 5 && nMinus >= 5);
                status = String.format(Locale.US, "Step 2: turn the ROBOT slowly left and right with the right stick, keeping the tag in view. "
                        + "%.0f of %.0f degrees, %d of %d pictures.", span, headingSpanDeg, headingPts.size(), headingPictures);
                if (nowMs - lastTagSeenMs > 1500) status = "Step 2: the tag is out of view: turn the robot back until the camera sees it.";
                if (span >= headingSpanDeg && headingPts.size() >= headingPictures && bothWays) finishHeading();
                return 0;
            default:
                return 0;
        }
    }

    private double scaleStep(double nowMs, int ticks, boolean tagSeen, double tagTx) {
        if (moving) {
            double err = targetTicks - ticks;
            int dir = (int) Math.signum(targetTicks - moveFromTicks);
            boolean arrived = Math.abs(err) <= 3 || (dir != 0 && Math.signum(err) != dir);
            if (Math.abs(ticks - lastProgressTicks) >= 2) { lastProgressTicks = ticks; lastProgressMs = nowMs; }
            if (!arrived && nowMs - lastProgressMs > 300) {          // F6: not moving
                if (boost < 0.12) { boost += 0.03; lastProgressMs = nowMs; }
                else blocked = true;
            }
            if (arrived || blocked || nowMs - moveStartMs > 3000) {
                moving = false;
                if (recentering) { startHeading(); return 0; }
                if (!Double.isNaN(pendingTarget) && !blocked) {        // F9: second half of a one-side approach
                    double tgt = pendingTarget; pendingTarget = Double.NaN;
                    return goTo(nowMs, ticks, tgt);
                }
                pendingTarget = Double.NaN;
                if (blocked) {                                        // hard stop: don't measure here, go back
                    blocked = false; boost = Math.max(0, boost - 0.06);
                    turnAround();
                    if (turns >= 2) { finishScale(); return 0; }
                    return goTo(nowMs, ticks, ticks + moveDir * stepTicks);
                }
                lastMoveDir = dir;
                beginCollect(nowMs);
                return 0;
            }
            double p = Math.max(minPower + boost, Math.min(maxPower, Math.abs(err) * 0.01));
            return Math.signum(err) * p;
        }
        if (collecting) {
            if (nowMs < settleUntilMs) return 0;
            if (Double.isNaN(collectStartMs)) collectStartMs = nowMs;
            if (tagSeen) { txSum += tagTx; txCount++; }
            if (txCount < picturesPerStop || nowMs - collectStartMs < averageMs) return 0;   // F1
            collecting = false;
            double tx = txSum / txCount;
            stops.add(new double[]{ticks, tx, lastMoveDir});
            status = String.format(Locale.US, "Step 1: the TURRET MOVES BY ITSELF - hands and wires clear, LB = STOP. "
                    + "Stop %d, turned around %d of 2, tag at %+.1f deg.", stops.size(), turns, tx);
            return planNextStop(nowMs, ticks, tx);
        }
        return 0;
    }

    private double moveFromTicks;

    private double goTo(double nowMs, int ticks, double target) {
        targetTicks = target; moveFromTicks = ticks; moving = true; moveStartMs = nowMs;
        lastProgressTicks = ticks; lastProgressMs = nowMs;
        return 0;
    }

    private double planNextStop(double nowMs, int ticks, double tx) {
        int n = stops.size();
        double txSpan = span(stops, 1);
        if (n >= 2) {
            if (txSpan < 1.5) stepTicks = Math.min(200, stepTicks * 2);   // F4: tag hasn't really moved yet
            else {
                double slope = fit(stops) ? fitSlope : simpleSlope(stops);
                if (!Double.isNaN(slope) && Math.abs(slope) > 1e-4)
                    stepTicks = Math.max(5, Math.min(Math.min(200, stepTicks * 2), stepDeg / Math.abs(slope)));
                double nextTx = tx + slope * moveDir * stepTicks;
                if (Math.abs(nextTx) > edgeDeg && Math.signum(nextTx) == Math.signum(slope * moveDir)) turnAround();
            }
            double prevTx = stops.get(n - 2)[1];
            // F5: already past the edge and the last move pushed it further out
            if (Math.abs(tx) > edgeDeg && Math.abs(tx) > Math.abs(prevTx) + 0.3 && stops.get(n - 1)[2] == moveDir) turnAround();
        }
        double next = ticks + moveDir * stepTicks;
        if ((moveDir > 0 && next > maxTicks) || (moveDir < 0 && next < minTicks)) {   // F7: outward only
            turnAround();
            next = ticks + moveDir * stepTicks;
        }
        boolean outward = (moveDir > 0 && next > maxTicks) || (moveDir < 0 && next < minTicks);
        if (turns >= 2 || stops.size() >= maxStops || outward) {
            finishScale(nowMs, ticks);
            return 0;
        }
        if (approachFromOneSide && moveDir < 0 && next - oneSideOverTicks >= minTicks) {
            pendingTarget = next;
            return goTo(nowMs, ticks, next - oneSideOverTicks);
        }
        return goTo(nowMs, ticks, next);
    }

    private void turnAround() { moveDir = -moveDir; turns++; }

    private void finishScale() { finishScale(Double.NaN, Integer.MIN_VALUE); }

    private void finishScale(double nowMs, int ticks) {
        if (stops.size() < 6 || span(stops, 1) < 8) {
            fail(String.format(Locale.US, "Not enough turret movement seen (%d stops, %.1f degrees). "
                    + "Park with the tag near the middle of the picture, away from the turret limits, and press RB again.",
                    stops.size(), span(stops, 1)));
            return;
        }
        if (!fit(stops)) { fail("Could not fit the readings. Press RB to try again."); return; }
        ticksPerDeg = 1.0 / Math.abs(fitSlope);
        turretDirOk = fitSlope * turretDir < 0;
        if (fitErrorDeg > 0.8 || slopeErrPct > maxSlopeErrPct) {
            fail(String.format(Locale.US, "The readings were too jumpy (%.2f degrees off the line, slope +-%.1f%%). Is the robot still and the camera "
                    + "tight? Press RB to try again.", fitErrorDeg, slopeErrPct));
            return;
        }
        // F8: re-center the tag before step 2
        if (!Double.isNaN(nowMs)) {
            double center = -(fitA) / fitSlope;   // ticks where tx ~ 0 (ignoring the slack offset)
            center = Math.max(minTicks, Math.min(maxTicks, center));
            recentering = true;
            goTo(nowMs, ticks, center);
            status = "Step 1 done: centering the tag for step 2.";
            return;
        }
        startHeading();
    }

    private void startHeading() {
        recentering = false;
        stage = Stage.HEADING;
        lastTagSeenMs = Double.POSITIVE_INFINITY;
        status = "Step 2: turn the ROBOT slowly left and right with the right stick, keeping the tag in view.";
    }

    /** Step 2 needs the robot to turn BOTH ways (at least one reversal) before deciding. */
    public boolean needBackAndForth = true;
    private double prevHeading = Double.NaN, prevHeadingMs; private int turnDir, lastTurnDir, reversals;

    private void finishHeading() {
        double k = simpleSlope(headingPts);
        if (needBackAndForth) { double[] f = fitGeneric(headingPts); if (f != null) k = f[1]; }
        headingRatio = k;
        headingSign = k > 0.5 ? 1 : (k < -0.5 ? -1 : 0);
        stage = Stage.DONE;
        status = "Done. RB = SAVE these results, LB = throw them away.";
    }

    private void beginCollect(double nowMs) {
        collecting = true; settleUntilMs = nowMs + settleMs; collectStartMs = Double.NaN; txSum = 0; txCount = 0;
    }

    private double fail(String why) {
        stage = Stage.FAILED;
        status = why;
        moving = false;
        ticksPerDeg = fitErrorDeg = backlashDeg = slopeErrPct = headingRatio = Double.NaN;  // nothing half-measured is kept
        headingSign = 0;
        turretDirOk = true;
        return 0;
    }

    /** LB: stop and forget everything measured. */
    public void discard() {
        stage = Stage.IDLE;
        moving = false;
        ticksPerDeg = fitErrorDeg = backlashDeg = slopeErrPct = headingRatio = Double.NaN;
        headingSign = 0;
        turretDirOk = true;
        status = "Camera check stopped; nothing saved. RB starts it again.";
    }

    /** After the results were saved. */
    public void saved(String what) {
        stage = Stage.IDLE;
        moving = false;
        status = what;
    }

    /** F2/F3: least squares tx = a + slope*ticks + h*dir over stops with a known approach side (dir != 0). */
    private boolean fit(List<double[]> all) {
        List<double[]> pts = new ArrayList<>();
        boolean plus = false, minus = false;
        for (double[] p : all) if (p[2] != 0) { pts.add(p); plus |= p[2] > 0; minus |= p[2] < 0; }
        boolean useDir = plus && minus && pts.size() >= 4;
        if (!useDir) {
            pts = new ArrayList<>();
            for (double[] p : all) if (p[2] != 0) pts.add(p);
            if (pts.size() < 4) pts = all;
        }
        int n = pts.size(), k = useDir ? 3 : 2;
        if (n < k + 1) return false;
        double[][] A = new double[k][k]; double[] b = new double[k];
        double t0 = 0; for (double[] p : pts) t0 += p[0]; t0 /= n;   // center ticks for conditioning
        for (double[] p : pts) {
            double[] x = useDir ? new double[]{1, p[0] - t0, p[2]} : new double[]{1, p[0] - t0};
            for (int i = 0; i < k; i++) { b[i] += x[i] * p[1]; for (int j = 0; j < k; j++) A[i][j] += x[i] * x[j]; }
        }
        double[][] inv = invert(A);
        if (inv == null) return false;
        double[] c = new double[k];
        for (int i = 0; i < k; i++) for (int j = 0; j < k; j++) c[i] += inv[i][j] * b[j];
        double ss = 0;
        for (double[] p : pts) {
            double r = p[1] - (c[0] + c[1] * (p[0] - t0) + (useDir ? c[2] * p[2] : 0));
            ss += r * r;
        }
        fitSlope = c[1]; fitA = c[0] - c[1] * t0; fitH = useDir ? c[2] : 0;
        fitErrorDeg = Math.sqrt(ss / n);
        double sigma2 = ss / Math.max(1, n - k);
        slopeErrPct = 100 * Math.sqrt(sigma2 * inv[1][1]) / Math.abs(c[1]);
        backlashDeg = 2 * Math.abs(fitH);
        return true;
    }

    /** {a, slope, dirTerm} for y = a + slope*x + dirTerm*dir (dir column may hold 0 for "not turning"). */
    private static double[] fitGeneric(List<double[]> pts) {
        boolean plus = false, minus = false;
        for (double[] p : pts) { plus |= p[2] > 0; minus |= p[2] < 0; }
        int k = plus && minus ? 3 : 2, n = pts.size();
        if (n < k + 1) return null;
        double[][] A = new double[k][k]; double[] b = new double[k];
        for (double[] p : pts) {
            double[] x = k == 3 ? new double[]{1, p[0], p[2]} : new double[]{1, p[0]};
            for (int i = 0; i < k; i++) { b[i] += x[i] * p[1]; for (int j = 0; j < k; j++) A[i][j] += x[i] * x[j]; }
        }
        double[][] inv = invert(A);
        if (inv == null) return null;
        double[] c = new double[3];
        for (int i = 0; i < k; i++) for (int j = 0; j < k; j++) c[i] += inv[i][j] * b[j];
        return c;
    }

    private static double[][] invert(double[][] m) {
        int n = m.length; double[][] a = new double[n][2 * n];
        for (int i = 0; i < n; i++) { System.arraycopy(m[i], 0, a[i], 0, n); a[i][n + i] = 1; }
        for (int col = 0; col < n; col++) {
            int piv = col; for (int r = col + 1; r < n; r++) if (Math.abs(a[r][col]) > Math.abs(a[piv][col])) piv = r;
            if (Math.abs(a[piv][col]) < 1e-12) return null;
            double[] t = a[col]; a[col] = a[piv]; a[piv] = t;
            double d = a[col][col]; for (int j = 0; j < 2 * n; j++) a[col][j] /= d;
            for (int r = 0; r < n; r++) if (r != col) { double f = a[r][col]; for (int j = 0; j < 2 * n; j++) a[r][j] -= f * a[col][j]; }
        }
        double[][] out = new double[n][n];
        for (int i = 0; i < n; i++) System.arraycopy(a[i], n, out[i], 0, n);
        return out;
    }

    static double simpleSlope(List<double[]> pts) {
        int n = pts.size();
        if (n < 2) return Double.NaN;
        double mx = 0, my = 0;
        for (double[] p : pts) { mx += p[0]; my += p[1]; }
        mx /= n; my /= n;
        double sxx = 0, sxy = 0;
        for (double[] p : pts) { sxx += (p[0] - mx) * (p[0] - mx); sxy += (p[0] - mx) * (p[1] - my); }
        return sxx < 1e-9 ? Double.NaN : sxy / sxx;
    }

    static double span(List<double[]> pts, int col) {
        if (pts.isEmpty()) return 0;
        double lo = Double.POSITIVE_INFINITY, hi = Double.NEGATIVE_INFINITY;
        for (double[] p : pts) { lo = Math.min(lo, p[col]); hi = Math.max(hi, p[col]); }
        return hi - lo;
    }

    /** One or two lines for the Driver Station. */
    public String resultText() {
        if (Double.isNaN(ticksPerDeg)) return "";
        String s = String.format(Locale.US, "Ticks per degree: %.2f (+-%.1f%%, fit error %.2f deg, belt slack %.1f deg)",
                ticksPerDeg, slopeErrPct, fitErrorDeg, backlashDeg);
        if (!Double.isNaN(headingRatio)) {
            s += String.format(Locale.US, "  |  Turn direction: %s (%+.2f)", headingSign > 0 ? "normal" : headingSign < 0
                    ? "MIRRORED" : "unclear: turn more, both ways", headingRatio);
        }
        // What it means, from BOTH steps (step 1 alone can't tell a mirrored camera from a reversed motor)
        if (!turretDirOk && headingSign < 0) {
            s += "  >> The CAMERA PICTURE looks mirrored (both tests flipped): check the Limelight's orientation setting. Do NOT change TURRET_DIR.";
        } else if (!turretDirOk && headingSign > 0) {
            s += "  >> The TURRET MOTOR runs backwards: check that D-pad RIGHT turns the turret RIGHT; if not, change TURRET_DIR in mechanumtest.java.";
        } else if (!turretDirOk) {
            s += "  >> Turret and camera disagree: do step 2 to find out which one is backwards.";
        } else if (headingSign < 0) {
            s += "  >> Turret and camera are mirrored compared with the gyro: the TeleOp will mirror the gyro for the aim.";
        }
        return s;
    }
}
