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
    private static final double MIN_DISTANCE = 1.25;
    private static final double MAX_DISTANCE = 5.0;

    // Separate LUTs for each field
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

    // Private constructor to populate LUTs
    private SOTMCalc() {
        // Distance to RPM
        rpmMap.put(1.25, 1000.0);
        rpmMap.put(1.75, 950.0);
        rpmMap.put(2.0, 1000.0);
        rpmMap.put(2.5, 1000.0);
        rpmMap.put(3.0, 1050.0);
        rpmMap.put(3.5, 1150.0);
        rpmMap.put(4.0, 1215.0);
        rpmMap.put(4.5, 1300.0);
        rpmMap.put(5.0, 1400.0);


        // Distance to hood angle
        hoodMap.put(1.25, 0.0);
        hoodMap.put(1.75, 7.5);
        hoodMap.put(2.0, 7.5);
        hoodMap.put(2.5, 7.5);
        hoodMap.put(3.0, 15.0);
        hoodMap.put(3.5, 15.0);
        hoodMap.put(4.0, 15.0);
        hoodMap.put(4.5, 20.0);
        hoodMap.put(5.0, 20.0);

        // distance to TOF
        tofMap.put(1.25, 0.94);
        tofMap.put(1.75, 0.83);
        tofMap.put(2.0, 0.86);
        tofMap.put(2.5, 0.95);
        tofMap.put(3.0, 0.86);
        tofMap.put(3.5, 1.01);
        tofMap.put(4.0, 1.21);
        tofMap.put(4.5, 1.11);
        tofMap.put(5.0, 1.24);
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
    /*public ShotConfigs calculateBoth(double distance, double requiredVelocity) {
        double rpmWeight = 0.7; // tuned to your robot
        double hoodWeight = 0.3;
        FullShooterParams baseline = getParameters(distance);

        // Compute baseline horizontal velocity based on RPM and hood
        //double baselineHorizVel = baseline.rpm / Math.cos(Math.toRadians(baseline.hoodAngle));

        // Ratio of target velocity to baseline
        //double velocityRatio = requiredVelocity / baselineHorizVel;

        // Split correction between RPM and hood using the weighing
        //double rpmFactor = Math.pow(velocityRatio, rpmWeight);
        //double hoodFactor = Math.pow(velocityRatio, hoodWeight);

        // Adjust RPM
        //double adjustedRpm = baseline.rpm * rpmFactor;

        // Adjust hood
        //double targetHorizFromHood = baselineHorizVel * hoodFactor;
        //double ratio = MathUtil.clamp(targetHorizFromHood / baselineHorizVel, 0.0, 1.0);
        //double adjustedHood = Math.toDegrees(Math.acos(ratio));
        FullShooterParams adjusted = getParameters(distanceToVirtualTarget);

        return new ShotConfigs(adjusted.hoodAngle, adjusted.rpm);
    }
        */
}
