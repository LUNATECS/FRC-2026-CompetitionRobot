package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * LUT-based Shoot-On-The-Move (SOTM) calculator.
 * Maps distance (meters) → shooter parameters (RPM, hood angle, time of flight).
 * Provides methods to calculate adjusted RPM and hood angle for moving targets.
 */
public class SOTMCalc {

    // Singleton instance
    private static SOTMCalc instance;

    public static SOTMCalc getInstance() {
        if (instance == null) {
            instance = new SOTMCalc();
        }
        return instance;
    }

    // LUT bounds
    private static final double MIN_DISTANCE = 1.5;
    private static final double MAX_DISTANCE = 5.0;

    // Separate LUTs for each field
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

    // Private constructor to populate LUTs
    private SOTMCalc() {
        // Distance → RPM
        rpmMap.put(1.5, 2800.0);
        rpmMap.put(2.0, 3100.0);
        rpmMap.put(2.5, 3400.0);
        rpmMap.put(3.0, 3650.0);
        rpmMap.put(3.5, 3900.0);
        rpmMap.put(4.0, 4100.0);
        rpmMap.put(4.5, 4350.0);
        rpmMap.put(5.0, 4550.0);

        // Distance → Hood Angle (degrees)
        hoodMap.put(1.5, 35.0);
        hoodMap.put(2.0, 38.0);
        hoodMap.put(2.5, 42.0);
        hoodMap.put(3.0, 46.0);
        hoodMap.put(3.5, 50.0);
        hoodMap.put(4.0, 54.0);
        hoodMap.put(4.5, 58.0);
        hoodMap.put(5.0, 62.0);

        // Distance → Time of Flight (seconds)
        tofMap.put(1.5, 0.38);
        tofMap.put(2.0, 0.45);
        tofMap.put(2.5, 0.52);
        tofMap.put(3.0, 0.60);
        tofMap.put(3.5, 0.68);
        tofMap.put(4.0, 0.76);
        tofMap.put(4.5, 0.85);
        tofMap.put(5.0, 0.94);
    }

    /** Record to hold interpolated shooter parameters */
    public record FullShooterParams(double rpm, double hoodAngle, double timeOfFlight) {}

    /** Utility: clamp distance to LUT bounds */
    private double clampDistance(double distance) {
        return MathUtil.clamp(distance, MIN_DISTANCE, MAX_DISTANCE);
    }

    /** Get interpolated shooter parameters at a given distance */
    public FullShooterParams getParameters(double distance) {
        distance = clampDistance(distance);
        double rpm = rpmMap.get(distance);
        double hood = hoodMap.get(distance);
        double tof = tofMap.get(distance);
        return new FullShooterParams(rpm, hood, tof);
    }

    /**
     * Adjust both RPM and hood for a required horizontal velocity.
     * @param distance The distance to target in meters
     * @param requiredVelocity The horizontal velocity needed to hit target
     * @return ShotConfigs containing adjusted hood and RPM
     */
    public ShotConfigs calculateBoth(double distance, double requiredVelocity) {
        double rpmWeight = 0.7; // tuned to your robot
        double hoodWeight = 0.3;
        FullShooterParams baseline = getParameters(distance);

        // Compute baseline horizontal velocity based on RPM and hood
        double baselineHorizVel = baseline.rpm / Math.cos(Math.toRadians(baseline.hoodAngle));

        // Ratio of target velocity to baseline
        double velocityRatio = requiredVelocity / baselineHorizVel;

        // Split correction between RPM and hood using the weighing
        double rpmFactor = Math.pow(velocityRatio, rpmWeight);
        double hoodFactor = Math.pow(velocityRatio, hoodWeight);

        // Adjust RPM
        double adjustedRpm = baseline.rpm * rpmFactor;

        // Adjust hood
        double targetHorizFromHood = baselineHorizVel * hoodFactor;
        double ratio = MathUtil.clamp(targetHorizFromHood / baselineHorizVel, 0.0, 1.0);
        double adjustedHood = Math.toDegrees(Math.acos(ratio));

        return new ShotConfigs(adjustedHood, adjustedRpm);
    }
}
