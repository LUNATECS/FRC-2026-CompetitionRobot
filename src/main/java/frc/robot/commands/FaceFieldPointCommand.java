package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class FaceFieldPointCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Translation2d> translationSupplier;
    private final Supplier<Translation2d> targetSupplier;
    private final SwerveRequest.FieldCentric driveRequest;

    private final PIDController thetaController =
        new PIDController(8.0, 0.0, 0.15);

    public FaceFieldPointCommand(
        CommandSwerveDrivetrain drivetrain,
        Supplier<Translation2d> translationSupplier,
        Supplier<Translation2d> targetSupplier,
        SwerveRequest.FieldCentric driveRequest
    ) {
        this.drivetrain = drivetrain;
        this.translationSupplier = translationSupplier;
        this.targetSupplier = targetSupplier;
        this.driveRequest = driveRequest;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(Math.toRadians(1.5));

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Rotation2d desiredHeading =
            drivetrain.getRotationToFacePoint(targetSupplier.get());

        double omega =
            thetaController.calculate(
                drivetrain.getPose().getRotation().getRadians(),
                desiredHeading.getRadians()
            );

        omega = MathUtil.clamp(omega, -3.0, 3.0);

        Translation2d translation = translationSupplier.get();

        drivetrain.setControl(
            driveRequest
                .withVelocityX(translation.getX())
                .withVelocityY(translation.getY())
                .withRotationalRate(omega)
        );
    }

    @Override
    public void end(boolean interrupted) {
        thetaController.reset();
    }
}


