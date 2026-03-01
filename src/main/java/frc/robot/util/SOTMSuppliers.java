package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubSystem;
import frc.robot.subsystems.ShooterSubSystem;

public class SOTMSuppliers {
    private Translation2d filteredVelocity = new Translation2d(0, 0);
    private final double alpha = 0.2;
    //reduce alpha for smoother but more delayed response, increase for more responsive but noisier output
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
            Pose2d robotPose = drivetrain.getPose();
            Pose2d launcherPose = robotPose.transformBy(SOTMCalc.ROBOT_TO_LAUNCHER);
            Translation2d launcherPos = launcherPose.getTranslation();
            Translation2d target = targetPos.get();

            // Distance to target
            double distance = launcherPos.getDistance(target);

            // Use SOTMCalc to get baseline time-of-flight
            return SOTMCalc.getInstance().getParameters(distance).timeOfFlight();
        };

        // Hood angle supplier
        hoodAngleSupplier = () -> {
            Translation2d target = targetPos.get();
            double distance = calculateLookaheadDistance(drivetrain, target);

            return SOTMCalc.getInstance()
                    //.getHybridSteppedHood(distance);
                    //OR
                    .getParameters(distance).hoodAngle();
        };

        shooterRPMSupplier = () -> {
            Translation2d target = targetPos.get();
            double distance = calculateLookaheadDistance(drivetrain, target);

            return SOTMCalc.getInstance()
                    .getParameters(distance).rpm();
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
    private double calculateLookaheadDistance(CommandSwerveDrivetrain drivetrain, Translation2d target) {
        Pose2d robotPose = drivetrain.getPose();
        Pose2d launcherPose = robotPose.transformBy(SOTMCalc.ROBOT_TO_LAUNCHER);
        Translation2d launcherPos = launcherPose.getTranslation();

        Translation2d filteredVel = getFilteredVelocity(drivetrain);

        double lookaheadDistance = launcherPos.getDistance(target);
        double tof = SOTMCalc.getInstance()
                .getParameters(lookaheadDistance).timeOfFlight();

        for (int i = 0; i < 10; i++) {
            tof = SOTMCalc.getInstance()
                    .getParameters(lookaheadDistance).timeOfFlight();

            Translation2d futureLauncherPos =
                launcherPos.plus(filteredVel.times(tof));

            lookaheadDistance =
                futureLauncherPos.getDistance(target);
        }

        return lookaheadDistance;
    }
}
