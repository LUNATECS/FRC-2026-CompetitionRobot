// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootOnTheMove;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootOnTheMoveCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Translation2d> translationSupplier;
    private final Supplier<Translation2d> targetSupplier;
    private final Supplier<Double> flightTimeSupplier;
    private final SwerveRequest.FieldCentric driveRequest;
    private final PIDController thetaController;
  /** Creates a new ShootOnTheMoveCommand. */
  public ShootOnTheMoveCommand(
        CommandSwerveDrivetrain drivetrain,
        Supplier<Translation2d> translationSupplier,
        Supplier<Translation2d> targetSupplier,
        Supplier<Double> flightTimeSupplier,
        SwerveRequest.FieldCentric driveRequest
  ) {
        this.drivetrain = drivetrain;
        this.translationSupplier = translationSupplier;
        this.targetSupplier = targetSupplier;
        this.flightTimeSupplier = flightTimeSupplier;
        this.driveRequest = driveRequest;
        thetaController=new PIDController(4.0, 0.0, 0.15);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(Math.toRadians(1.5));

        addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
public void execute() {
    Pose2d robotPose = drivetrain.getPose();

    // Use suppliers to get virtual target
    
        Translation2d robotPos = robotPose.getTranslation();
        Translation2d target = targetSupplier.get();
        double flightTime = flightTimeSupplier.get();
        Translation2d virtualTarget = target.minus(
            drivetrain.getFieldVelocity().times(flightTime)
                //.plus(drivetrain.getFieldAcceleration().times(0.5 * flightTime * flightTime))
        );
  

    // Rotate drivetrain to face virtual target
    Rotation2d desiredHeading = drivetrain.getRotationToFacePoint(virtualTarget);
    double omega = thetaController.calculate(
            robotPose.getRotation().getRadians(),
            desiredHeading.getRadians()
    );
    omega = MathUtil.clamp(omega, -3.0, 3.0);

    // Get translational movement
    Translation2d translation = translationSupplier.get();

    RobotContainer.virtualTargetField.setRobotPose(new Pose2d(virtualTarget, Rotation2d.fromDegrees(0))); 

    double translationXClamping = MathUtil.clamp(translation.getX(), -1.5, 1.5);
    double translationYClamping = MathUtil.clamp(translation.getY(), -1.5, 1.5);

    drivetrain.setControl(
            driveRequest.withVelocityX(translationXClamping)
                        .withVelocityY(translationYClamping)
                        .withRotationalRate(omega)
    );
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        thetaController.reset();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
