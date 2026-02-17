package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubSystem;
import frc.robot.subsystems.ShooterSubSystem;

public class SOTMSuppliers {
    private Translation2d filteredVelocity = new Translation2d(0, 0);
    private final double alpha = 0.2;
    public final Supplier<Double> hoodAngleSupplier;
    public final Supplier<Double> shooterRPMSupplier;
    public final Supplier<Double> flightTimeSupplier;

    public SOTMSuppliers(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubSystem shooter,
            HoodSubSystem hood,
            Supplier<Translation2d> targetPos
    ) {
        

        // Flight time supplier
        flightTimeSupplier = () -> {
            Translation2d robotPos = drivetrain.getPose().getTranslation();
            Translation2d target = targetPos.get();

            // Distance to target
            double distance = robotPos.getDistance(target);

            // Use SOTMCalc to get baseline time-of-flight
            return SOTMCalc.getInstance().getParameters(distance).timeOfFlight();
        };

        // Hood angle supplier
        hoodAngleSupplier = () -> {
            Translation2d robotPos = drivetrain.getPose().getTranslation();
            Translation2d target = targetPos.get();

            // Lead compensation
            double flightTime = flightTimeSupplier.get();
            filteredVelocity = getFilteredVelocity(drivetrain);
            Translation2d leadOffset = filteredVelocity.times(flightTime);
                    //.plus(drivetrain.getFieldAcceleration().times(0.5 * flightTime * flightTime));
            Translation2d virtualTarget = target.minus(leadOffset);

            double distanceToVirtualTarget = robotPos.getDistance(virtualTarget);

            // Required horizontal velocity to reach the virtual target
            double requiredVelocity = distanceToVirtualTarget / flightTime;

            // Use SOTMCalc to get hood angle
            return SOTMCalc.getInstance().getParameters(distanceToVirtualTarget).hoodAngle();
        };

        shooterRPMSupplier = () -> {
            Translation2d robotPos = drivetrain.getPose().getTranslation();
            Translation2d target = targetPos.get();

            // Lead compensation
            double flightTime = flightTimeSupplier.get();
            filteredVelocity = getFilteredVelocity(drivetrain);
            Translation2d leadOffset = filteredVelocity.times(flightTime);
                    //.plus(drivetrain.getFieldAcceleration().times(0.5 * flightTime * flightTime));
            Translation2d virtualTarget = target.minus(leadOffset);

            double distanceToVirtualTarget = robotPos.getDistance(virtualTarget);

            // Required horizontal velocity to reach the virtual target
            double requiredVelocity = distanceToVirtualTarget / flightTime;

            // Use SOTMCalc to get RPM
            return SOTMCalc.getInstance().getParameters(distanceToVirtualTarget).rpm();
        };
    }

    private Translation2d getFilteredVelocity(CommandSwerveDrivetrain drivetrain) {
    Translation2d rawVel = drivetrain.getFieldVelocity();

    filteredVelocity = new Translation2d(
        filteredVelocity.getX() + alpha * (rawVel.getX() - filteredVelocity.getX()),
        filteredVelocity.getY() + alpha * (rawVel.getY() - filteredVelocity.getY())
    );

    return filteredVelocity;
}
}
