package frc.robot.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveOverBumpCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric driveRequest;

    // CONFIGURABLE VALUES FROM INPUTS
    private final double distanceMeters;
    private final double driveSpeed;
    private final Rotation2d targetHeading;

    private Pose2d startPose;

    private final PIDController headingPID;

    public DriveOverBumpCommand(
        CommandSwerveDrivetrain drivetrain,
        double distanceMeters,
        double driveSpeedMetersPerSec,
        Rotation2d targetHeading,
        SwerveRequest.FieldCentric driveRequest
    ) {
        this.drivetrain = drivetrain;
        this.distanceMeters = distanceMeters;
        this.driveSpeed = driveSpeedMetersPerSec;
        this.targetHeading = targetHeading;
        this.driveRequest = driveRequest;
        
        headingPID = new PIDController(4.0, 0.0, 0.15);

        headingPID.enableContinuousInput(-Math.PI, Math.PI);
        headingPID.setTolerance(Math.toRadians(2.0));

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        startPose = drivetrain.getPose();
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();

        double traveled =
            currentPose.getTranslation()
                       .getDistance(startPose.getTranslation());

        double omega =
            headingPID.calculate(
                currentPose.getRotation().getRadians(),
                targetHeading.getRadians()
            );

        omega = MathUtil.clamp(omega, -3.0, 3.0);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(driveSpeed)
                .withVelocityY(0.0)
                .withRotationalRate(omega)
        );
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = drivetrain.getPose();
        return currentPose.getTranslation()
            .getDistance(startPose.getTranslation()) >= distanceMeters;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
        headingPID.reset();
    }
}

