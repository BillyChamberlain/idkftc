package org.firstinspires.ftc.teamcode;

/**
 * TurretAimer keeps the turret pointed at the goal, even when the camera loses
 * the AprilTags.
 *
 * How it works:
 *  1. When the camera sees the goal, we work out WHERE the goal is, as a point
 *     in the Pinpoint odometry frame: robot position + (turret angle + tx)
 *     direction + camera distance. We use where the robot and turret were when
 *     the picture was TAKEN, since a Limelight frame is ~30-60 ms old.
 *  2. Every loop, odometry tells us where the robot is now, so we can compute
 *     the turret angle that points at that goal point, whether the tags are
 *     visible or not. Each new sighting re-anchors the goal point, so odometry
 *     drift never builds up for long.
 *  3. Feed-forward: when the robot turns or drives sideways, we spin the turret
 *     at the rate the goal's direction is changing, instead of waiting to fall
 *     behind.
 *  4. If the goal is in the turret's blind spot (past the soft limits), the
 *     turret waits at one limit, and switches to the other limit once the goal
 *     is closer to coming out on that side, so it's already there when it does.
 *  5. FASTEST WAY (added after Thursday): in the blind spot, the turret waits on
 *     the side where it will be pointing at the goal again SOONEST: how long the
 *     goal takes to come out there (from how fast the robot is turning) vs. how
 *     long the turret takes to get there. Everything else is Thursday's code.
 *
 * Sign conventions used everywhere in this class:
 *   turret degrees   + = turret turns RIGHT (clockwise seen from above)
 *   robot heading    + = robot turns LEFT (counter-clockwise), the Pinpoint default
 *   odometry x, y      = Pinpoint frame in inches (x forward, y left at start)
 *   tx               + = target is right of the crosshair, the Limelight default
 *
 * Plain Java with no hardware, so it can be tested on a laptop.
 */
public class TurretAimer {

    // ---------------- Tuning ----------------
    /** Turret speed (deg/s) per degree of aim error. Higher = snappier, too high = wobble. */
    public double kP = 18.0;
    /** Fastest the turret is allowed to spin while auto-aiming. */
    public double maxSpeedDegPerSec = 700.0;
    /** Inside this error we stop correcting (prevents buzzing back and forth). */
    public double deadbandDeg = 0.5;
    /** Builds up extra speed while the turret keeps trailing the goal (removes steady lag). */
    public double kI = 5.0;
    /** Most extra speed kI may add, deg/s. */
    public double maxIDegPerSec = 150.0;
    /** Without a distance, keep aiming at the last seen direction this long. */
    public double directionMemoryMs = 1500.0;
    /** With a distance, keep tracking the goal point with odometry this long. */
    public double odometryMemoryMs = 1e12;  // never forget: the goal does not move, odometry keeps aiming
    /** Track the goal point with odometry. Turn off if the Pinpoint x/y are wrong. */
    public boolean useOdometry = true;
    /** How much closer the other limit must be before the turret switches sides in the blind spot. */
    public double blindSpotHysteresisDeg = 12.0;
    /** A new sighting further than this from the stored goal point replaces it instead of averaging. */
    public double snapDistanceIn = 36.0;
    /** How much of each new sighting to blend in (1 = trust the newest frame completely). */
    public double sightingBlend = 0.25;
    /** A far-off sighting must repeat this many frames in a row before it moves the goal (ignores glitches). */
    public int outlierFramesToSnap = 5;
    /** Seconds of ball flight to lead the goal by (shoot on the move). 0 = aim straight at it. */
    public double leadTimeSec = 0;
    /** Fastest way: pick the blind-spot side by time (off = Thursday's nearest-side rule only). */
    public boolean fastestWay = true;
    /** Fastest way: trust the goal's predicted motion only when it moves at least this fast, deg/s. */
    public double fastestMinRateDegPerSec = 20.0;
    /** Fastest way: switch sides only if the other side is at least this much sooner, seconds. */
    public double fastestMarginSec = 0.25;
    /** Where the turret's pivot is on the robot, relative to the Pinpoint's center (inches). */
    public double turretForwardIn = 0.0, turretLeftIn = 0.0;

    private final double minDeg, maxDeg;
    private double iTerm = 0, lastUpdateMs = Double.NaN;
    private int outliers = 0;

    // History of robot/turret state, to look up where we were when a camera frame was captured
    private static final int HISTORY = 64;
    private final double[] histT = new double[HISTORY];
    private final double[] histTurret = new double[HISTORY];
    private final double[] histHeading = new double[HISTORY];
    private final double[] histX = new double[HISTORY];
    private final double[] histY = new double[HISTORY];
    private int histCount = 0, histNext = 0;

    // Latest state and smoothed rates
    private double curT, curTurret, curHeading, curX, curY;
    private double headingRate, velX, velY;  // deg/s, in/s (odometry frame)

    // What we know about the goal
    private boolean hasDirection = false, hasPoint = false;
    private double goalDirDeg;               // odometry-frame direction (CCW+), from the turret pivot
    private double goalX, goalY;             // odometry-frame point (inches)
    private double lastSeenMs = Double.NEGATIVE_INFINITY;

    // Blind-spot state: -1 parked at min limit, +1 parked at max limit, 0 not parked
    private int parkedSide = 0;
    private boolean lastPickBlind = false;
    private int lastPickSide = 0;
    private double pickRate = 0;  // the goal's bearing rate when choosing (fastest way)

    /** @param minDeg @param maxDeg the turret's soft limits in turret degrees */
    public TurretAimer(double minDeg, double maxDeg) {
        this.minDeg = Math.min(minDeg, maxDeg);
        this.maxDeg = Math.max(minDeg, maxDeg);
    }

    /**
     * Call once per loop BEFORE seeTarget/update.
     * @param headingDeg unnormalized heading (doesn't wrap at 180)
     * @param xIn @param yIn odometry position in inches
     */
    public void record(double nowMs, double turretDeg, double headingDeg, double xIn, double yIn) {
        if (histCount > 0) {
            double dt = (nowMs - curT) / 1000.0;
            if (dt > 0.001 && dt < 0.5) {
                double a = 0.6;  // smoothing (higher = reacts faster to turns)
                headingRate += a * ((headingDeg - curHeading) / dt - headingRate);
                velX += a * ((xIn - curX) / dt - velX);
                velY += a * ((yIn - curY) / dt - velY);
            }
        }
        curT = nowMs; curTurret = turretDeg; curHeading = headingDeg; curX = xIn; curY = yIn;

        histT[histNext] = nowMs;
        histTurret[histNext] = turretDeg;
        histHeading[histNext] = headingDeg;
        histX[histNext] = xIn;
        histY[histNext] = yIn;
        histNext = (histNext + 1) % HISTORY;
        if (histCount < HISTORY) histCount++;
    }

    /**
     * Call when the camera sees the goal.
     * @param txDeg      angle from crosshair to the goal center (+ = right)
     * @param frameAgeMs how old the picture is
     * @param distanceIn camera-to-goal distance, or NaN if unknown
     */
    public void seeTarget(double nowMs, double txDeg, double frameAgeMs, double distanceIn) {
        double t = nowMs - Math.max(0, frameAgeMs);
        double turret = lookup(histTurret, t);
        double heading = lookup(histHeading, t);
        double x = lookup(histX, t), y = lookup(histY, t);

        goalDirDeg = heading - (turret + txDeg);
        hasDirection = true;
        lastSeenMs = t;

        if (!Double.isNaN(distanceIn) && distanceIn > 1) {
            double h = Math.toRadians(heading);
            double px = x + turretForwardIn * Math.cos(h) - turretLeftIn * Math.sin(h);
            double py = y + turretForwardIn * Math.sin(h) + turretLeftIn * Math.cos(h);
            double d = Math.toRadians(goalDirDeg);
            double gx = px + distanceIn * Math.cos(d);
            double gy = py + distanceIn * Math.sin(d);
            boolean far = hasPoint && Math.hypot(gx - goalX, gy - goalY) > snapDistanceIn;
            outliers = far ? outliers + 1 : 0;
            if (!hasPoint || outliers >= outlierFramesToSnap) {
                goalX = gx;
                goalY = gy;
                outliers = 0;
            } else if (!far) {
                goalX += sightingBlend * (gx - goalX);
                goalY += sightingBlend * (gy - goalY);
            }
            hasPoint = true;
        }
    }

    private boolean pointValid(double nowMs) {
        return useOdometry && hasPoint && nowMs - lastSeenMs <= odometryMemoryMs;
    }

    private boolean directionValid(double nowMs) {
        return hasDirection && nowMs - lastSeenMs <= directionMemoryMs;
    }

    /** True while we know where the goal is (from the camera, odometry, or recent heading). */
    public boolean isTracking(double nowMs) {
        return pointValid(nowMs) || directionValid(nowMs);
    }

    /** "CAMERA", "ODOMETRY", "HEADING" or "" (lost), for the HUD. */
    public String source(double nowMs) {
        if (nowMs - lastSeenMs < 200 && hasDirection) return "CAMERA";
        if (pointValid(nowMs)) return "ODOMETRY";
        if (directionValid(nowMs)) return "HEADING";
        return "";
    }

    // ---- Small read-outs for the TeleOp / HUD (added after Thursday; they don't change the aiming) ----

    /** Turret angle (+ = right, not limited) toward where the goal was last seen, from the heading. NaN if never seen. */
    public double lastSeenBearingDeg() { return hasDirection ? wrap180(curHeading - goalDirDeg) : Double.NaN; }

    /** The robot's position was reset (LT+RT): forget its recent motion so the jump isn't read as speed. */
    public void resetMotion() {
        histCount = 0;
        headingRate = velX = velY = 0;
        iTerm = 0;
    }

    /** Where to aim: the goal, shifted against the robot's motion so the moving ball lands in it. */
    private double aimX() { return goalX - clamp(velX, -100, 100) * clamp(leadTimeSec, 0, 1.5); }
    private double aimY() { return goalY - clamp(velY, -100, 100) * clamp(leadTimeSec, 0, 1.5); }

    /** Turret angle (+ = right) that points at the goal, before limits. */
    private double rawBearingDeg(double nowMs) {
        if (pointValid(nowMs)) {
            double[] p = pivot();
            double dirDeg = Math.toDegrees(Math.atan2(aimY() - p[1], aimX() - p[0]));
            return wrap180(curHeading - dirDeg);
        }
        return wrap180(curHeading - goalDirDeg);
    }

    /** Turret angle that points at the goal right now, kept inside the soft limits. */
    public double desiredDeg(double nowMs) {
        pickRate = bearingRateDegPerSec(nowMs);
        return pickReachable(rawBearingDeg(nowMs), curTurret);
    }

    /** True when the goal is in the blind spot and the turret is waiting at a limit. */
    public boolean isParked(double nowMs) {
        if (!isTracking(nowMs)) return false;
        desiredDeg(nowMs);
        return lastPickBlind;
    }

    /** Aim error in degrees (desired minus current), or 0 when not tracking. */
    public double errorDeg(double nowMs) {
        if (!isTracking(nowMs)) return 0;
        return desiredDeg(nowMs) - curTurret;
    }

    /** Turret speed command in deg/s (+ = turn right). Returns 0 when not tracking. */
    public double update(double nowMs) {
        if (!isTracking(nowMs)) {
            parkedSide = 0;
            iTerm = 0;
            return 0;
        }
        double desired = desiredDeg(nowMs);
        parkedSide = lastPickBlind ? lastPickSide : 0;

        double rate = lastPickBlind ? 0 : bearingRateDegPerSec(nowMs);
        double error = desired - curTurret;

        double dt = Double.isNaN(lastUpdateMs) ? 0 : (nowMs - lastUpdateMs) / 1000.0;
        lastUpdateMs = nowMs;
        if (lastPickBlind || Math.abs(error) > 15 || dt <= 0 || dt > 0.25) {
            iTerm *= 0.5;  // parked, far off, or a long gap: let it go
        } else {
            iTerm = clamp(iTerm + kI * error * dt, -maxIDegPerSec, maxIDegPerSec);
        }

        double cmd = (Math.abs(error) <= deadbandDeg ? 0 : kP * error) + iTerm + rate;
        return clamp(cmd, -maxSpeedDegPerSec, maxSpeedDegPerSec);
    }

    /** How fast the goal's direction is moving relative to the robot (+ = rightward), deg/s. */
    double bearingRateDegPerSec(double nowMs) {
        double rate = headingRate;  // robot turning left -> goal swings right
        if (pointValid(nowMs)) {
            double h = Math.toRadians(curHeading);
            double cos = Math.cos(h), sin = Math.sin(h);
            double[] p = pivot();
            double dx = goalX - p[0], dy = goalY - p[1];
            double f = dx * cos + dy * sin;       // goal ahead of robot
            double l = -dx * sin + dy * cos;      // goal left of robot
            double vf = velX * cos + velY * sin;  // robot velocity, robot frame
            double vl = -velX * sin + velY * cos;
            double r2 = f * f + l * l;
            if (r2 > 36) rate += Math.toDegrees((f * vl - l * vf) / r2);
        }
        return rate;
    }

    private double[] pivot() {
        double h = Math.toRadians(curHeading);
        return new double[]{
                curX + turretForwardIn * Math.cos(h) - turretLeftIn * Math.sin(h),
                curY + turretForwardIn * Math.sin(h) + turretLeftIn * Math.cos(h)};
    }

    /**
     * The same direction can be reached as a, a+360 or a-360. Pick the one inside
     * the soft limits that is closest to where the turret is now. If the goal is
     * in the blind spot, wait at a limit: the one the goal is closer to coming out
     * of, with hysteresis so the turret doesn't flip back and forth.
     */
    double pickReachable(double angle, double current) {
        angle = wrap180(angle);
        double best = Double.NaN, bestDist = Double.POSITIVE_INFINITY;
        for (int k = -2; k <= 2; k++) {
            double a = angle + 360.0 * k;
            if (a < minDeg || a > maxDeg) continue;
            double d = Math.abs(a - current);
            if (d < bestDist) { best = a; bestDist = d; }
        }
        if (!Double.isNaN(best)) {
            lastPickBlind = false;
            lastPickSide = 0;
            return best;
        }

        // Blind spot runs clockwise from maxDeg to minDeg + 360
        double a = maxDeg + (((angle - maxDeg) % 360.0) + 360.0) % 360.0;
        double toMax = a - maxDeg;
        double toMin = (minDeg + 360.0) - a;
        int side;
        if (parkedSide > 0)      side = (toMin + blindSpotHysteresisDeg < toMax) ? -1 : 1;
        else if (parkedSide < 0) side = (toMax + blindSpotHysteresisDeg < toMin) ? 1 : -1;
        else                     side = toMax <= toMin ? 1 : -1;
        if (fastestWay && Math.abs(pickRate) >= fastestMinRateDegPerSec) {
            // Fastest way: when will the turret point at the goal again on each side?
            // The goal comes out at min if its bearing keeps growing (rate > 0), at max if shrinking;
            // the turret also has to get there.
            double speed = Math.max(30, maxSpeedDegPerSec);
            double goalToMin = pickRate > 0 ? toMin / pickRate : Double.POSITIVE_INFINITY;
            double goalToMax = pickRate < 0 ? toMax / -pickRate : Double.POSITIVE_INFINITY;
            double tMin = Math.max(goalToMin, Math.abs(current - minDeg) / speed);
            double tMax = Math.max(goalToMax, Math.abs(current - maxDeg) / speed);
            int keep = parkedSide != 0 ? parkedSide : side;
            if (tMin + fastestMarginSec < tMax) side = -1;
            else if (tMax + fastestMarginSec < tMin) side = 1;
            else side = keep;
        }
        lastPickBlind = true;
        lastPickSide = side;
        return side > 0 ? maxDeg : minDeg;
    }

    /** Linear interpolation in the history buffer at time t. */
    double lookup(double[] values, double t) {
        if (histCount == 0) return 0;
        int newest = (histNext - 1 + HISTORY) % HISTORY;
        if (t >= histT[newest]) return values[newest];
        int idx = newest;
        for (int i = 1; i < histCount; i++) {
            int prev = (idx - 1 + HISTORY) % HISTORY;
            if (histT[prev] <= t) {
                double span = histT[idx] - histT[prev];
                double f = span <= 0 ? 0 : (t - histT[prev]) / span;
                return values[prev] + f * (values[idx] - values[prev]);
            }
            idx = prev;
        }
        return values[idx]; // older than everything we remember
    }

    static double wrap180(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // =====================================================================
    //  Cluster center finder
    // =====================================================================

    /**
     * BIOBUZZ goals are marked by a row of 4 AprilTags. These are each tag's
     * sideways offset (inches) from the cluster center, for IDs base+0..base+3,
     * from the FTC SDK 12.0 AprilTag database.
     */
    public static final double[] MEMBER_X_IN = {-6.5, -2.75, 2.75, 6.5};

    /**
     * Finds the tx of the cluster CENTER from whichever of its tags are visible.
     *
     * If robots block some tags, averaging the visible ones aims off-center
     * (6.5 inches at 5 feet is about 6 degrees). With 2+ tags visible we fit a
     * line tx = center + slope * offset; the slope also absorbs distance and which
     * way the IDs run, so no extra measurements are needed. With 1 tag, we reuse
     * the slope from the last 2+ tag frame if it is recent.
     */
    public static class ClusterFit {
        private double slope = Double.NaN;          // degrees per inch of tag offset
        private double slopeTimeMs = Double.NEGATIVE_INFINITY;
        public double slopeMemoryMs = 1500.0;

        /**
         * @param member member index 0..3 (tag ID minus cluster base) for each visible tag
         * @param tx     tx of each visible tag, same order
         * @param n      how many entries are filled in
         */
        public double centerTx(double nowMs, int[] member, double[] tx, int n) {
            if (n <= 0) return Double.NaN;
            double meanX = 0, meanTx = 0;
            for (int i = 0; i < n; i++) { meanX += MEMBER_X_IN[member[i]]; meanTx += tx[i]; }
            meanX /= n;
            meanTx /= n;

            if (n >= 2) {
                double sxx = 0, sxt = 0;
                for (int i = 0; i < n; i++) {
                    double dx = MEMBER_X_IN[member[i]] - meanX;
                    sxx += dx * dx;
                    sxt += dx * (tx[i] - meanTx);
                }
                double s = sxx > 1e-9 ? sxt / sxx : Double.NaN;
                // Sanity check: even at 1 foot away, 1 inch is under 5 degrees
                if (!Double.isNaN(s) && Math.abs(s) < 5.0) {
                    slope = s;
                    slopeTimeMs = nowMs;
                    return meanTx - s * meanX;
                }
                return meanTx;
            }
            if (!Double.isNaN(slope) && nowMs - slopeTimeMs <= slopeMemoryMs) {
                return tx[0] - slope * MEMBER_X_IN[member[0]];
            }
            return tx[0];
        }
    }
}
