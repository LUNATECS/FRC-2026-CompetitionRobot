package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubSystem;
import frc.robot.subsystems.ShooterSubSystem;

public class ShootOnTheMoveSuppliers {
    public final Supplier<Double> hoodAngleSupplier;
    public final Supplier<Double> shooterRPMSupplier;
    public final Supplier<Double> flightTimeSupplier;

    public ShootOnTheMoveSuppliers(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubSystem shooter,
            HoodSubSystem hood,
            Supplier<Translation2d> targetPos
    ) {

        // Flight time supplier
        flightTimeSupplier = () -> {
            Translation2d robotPos = drivetrain.getPose().getTranslation();
            Translation2d target = targetPos.get();

            double rpm = shooter.getRPM();
            double projectileSpeed = CommandSwerveDrivetrain.calculateProjectileSpeed(rpm);
            double hoodRad = Math.toRadians(hood.getHoodDegrees());

            double deltaX = robotPos.getDistance(target);
            double deltaY = 1.143; // assume same height
            return drivetrain.computeFlightTime(projectileSpeed, deltaX, deltaY, hoodRad);
        };

        // Hood angle supplier
        hoodAngleSupplier = () -> {
            Translation2d robotPos = drivetrain.getPose().getTranslation();
            Translation2d target = targetPos.get();

            double rpm = shooter.getRPM();
            double projectileSpeed = CommandSwerveDrivetrain.calculateProjectileSpeed(rpm);
            double hoodRad = Math.toRadians(hood.getHoodDegrees());

            // Lead compensation
            double flightTime = flightTimeSupplier.get();
            Translation2d leadOffset = drivetrain.getFieldVelocity().times(flightTime)
                    .plus(drivetrain.getFieldAcceleration().times(0.5 * flightTime * flightTime));
            Translation2d virtualTarget = target.minus(leadOffset);

            double distanceToTarget = robotPos.getDistance(virtualTarget);
            return ShotCalc.getShot(distanceToTarget).hoodDeg;
        };

        // Shooter RPM supplier
        shooterRPMSupplier = () -> {
            Translation2d robotPos = drivetrain.getPose().getTranslation();
            Translation2d target = targetPos.get();

            double hoodDeg = hoodAngleSupplier.get();
            double hoodRad = Math.toRadians(hoodDeg);

            // Distance to virtual target for lead compensation
            double flightTime = flightTimeSupplier.get();
            Translation2d leadOffset = drivetrain.getFieldVelocity().times(flightTime)
                    .plus(drivetrain.getFieldAcceleration().times(0.5 * flightTime * flightTime));
            Translation2d virtualTarget = target.minus(leadOffset);

            double distanceToTarget = robotPos.getDistance(virtualTarget);
            return ShotCalc.getShot(distanceToTarget).rpm;
        };
    }
}
