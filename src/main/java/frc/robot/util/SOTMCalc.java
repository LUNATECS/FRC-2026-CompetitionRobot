package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.commands.HoodDynamicPointCommand;

/**
 * LUT-based Shoot-On-The-Move (SOTM) calculator.
 * Maps distance (meters) → shooter parameters (RPM, hood angle, time of flight).
 * Provides methods to calculate adjusted RPM and hood angle for moving targets.
 */
public class SOTMCalc {

    // Singleton instance
    private static SOTMCalc instance;

    public static synchronized SOTMCalc getInstance() {
        if (instance == null) {
            instance = new SOTMCalc();
        }
        return instance;
    }

    // LUT bounds
    private static final double MIN_DISTANCE = 1.5;
    private static final double MAX_DISTANCE = 12.0;

    // Separate LUTs for each field
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();
    public static final Transform2d ROBOT_TO_LAUNCHER =
    new Transform2d(
        new Translation2d(0.1607, 0.0), // 6.325 inches forward
        new Rotation2d()
    );

    // Private constructor to populate LUTs
    private SOTMCalc() {
        // Distance to RPM
        rpmMap.put(1.5, 2800.0);
        rpmMap.put(1.75, 2900.0);
        rpmMap.put(2.0, 3000.0);
        rpmMap.put(2.5, 3150.0);
        rpmMap.put(3.0, 3400.0);
        rpmMap.put(3.5, 3460.0);
        rpmMap.put(4.0, 3660.0);
        rpmMap.put(4.5, 3850.0);
        rpmMap.put(5.0, 4100.0);
        rpmMap.put(6.0, 4300.0);
        rpmMap.put(12.0, 6500.0);

        // Distance to hood angle
        hoodMap.put(1.5, 2.5);
        hoodMap.put(1.75, 5.0);
        hoodMap.put(2.0, 7.5);
        hoodMap.put(2.5, 7.5);
        hoodMap.put(3.0, 7.5);
        hoodMap.put(3.5, 15.0);
        hoodMap.put(4.0, 15.0);
        hoodMap.put(4.5, 15.0);
        hoodMap.put(5.0, 18.0);
        hoodMap.put(6.0, 18.0);
        hoodMap.put(12.0, 27.5);

        // distance to TOF
        tofMap.put(1.5, 0.84);
        tofMap.put(1.75, 0.9);
        tofMap.put(2.0, 0.97);
        tofMap.put(2.5, 1.00);
        tofMap.put(3.0, 1.07);
        tofMap.put(3.5, 1.14);
        tofMap.put(4.0, 1.2);
        tofMap.put(4.5, 1.24);
        tofMap.put(5.0, 1.28);
        tofMap.put(6.0, 1.5);
        tofMap.put(12.0, 2.625);

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


        public double getSteppedHood(double distance) {
        distance = clampDistance(distance);

        // Manually define your ranges
        if (distance < 1.75) {
            return 0.0;
        } else if (distance < 3.0) {
            return 7.5;
        } else if (distance < 4.5) {
            return 15.0;
        } else {
            return 20.0;
        }
    }

    public double getHybridSteppedHood(double distance) {
        distance = clampDistance(distance);

        // Transition zones
        if (distance >= 1.25 && distance <= 1.75) {
            double t = (distance - 1.25) / (1.75 - 1.25);
            return 0.0 + t * (7.5 - 0.0);
        } else if (distance >= 2.5 && distance <= 3.0) {
            double t = (distance - 2.5) / (3.0 - 2.5);
            return 7.5 + t * (15.0 - 7.5);
        } else if (distance >= 4.0 && distance <= 4.5) {
            double t = (distance - 4.0) / (4.5 - 4.0);
            return 15.0 + t * (20.0 - 15.0);
        } else {
            // Outside transition zones → stepped LUT
            return hoodMap.get(distance);
        }
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
