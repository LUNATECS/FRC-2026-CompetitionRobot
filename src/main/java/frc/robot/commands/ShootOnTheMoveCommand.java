// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubSystem;
import frc.robot.subsystems.ShooterSubSystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootOnTheMoveCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Supplier<Translation2d> translationSupplier;
  private final Supplier<Translation2d> targetSupplier;
  private final Supplier<Double> timeOfFlightSupplier;
  private final SwerveRequest.FieldCentric driveRequest;
  private final PIDController thetaController;
  private final ShooterSubSystem shooter;
  private final HoodSubSystem hood;
  private final Supplier<Double> shooterRPMSupplier;
  private final Supplier<Double> hoodAngleDegSupplier;
  /** Creates a new ShootOnTheMoveCommand. */
  public ShootOnTheMoveCommand(
        CommandSwerveDrivetrain drivetrain,
        Supplier<Translation2d> translationSupplier,
        Supplier<Translation2d> targetSupplier,
        SwerveRequest.FieldCentric driveRequest,
        Supplier<Double> timeOfFlightSupplier,
        Supplier<Double> shooterRPMSupplier,
        Supplier<Double> hoodAngleDegSupplier,
        ShooterSubSystem shooter,
        HoodSubSystem hood
  ) {
        this.drivetrain = drivetrain;
        this.translationSupplier = translationSupplier;
        this.targetSupplier = targetSupplier;
        this.driveRequest = driveRequest;
        this.shooter = shooter;
        this.hood = hood;
        this.timeOfFlightSupplier = timeOfFlightSupplier;
        this.shooterRPMSupplier = shooterRPMSupplier;
        this.hoodAngleDegSupplier = hoodAngleDegSupplier;
        thetaController=new PIDController(8.0, 0.0, 0.15);

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
        Translation2d robotPos = robotPose.getTranslation();
        Translation2d targetPos = targetSupplier.get();

        // Get current shot parameters
        double rpm = shooterRPMSupplier.get();
        double hoodDeg = hoodAngleDegSupplier.get();

        // Convert RPM to projectile speed
        double projectileSpeed =
            CommandSwerveDrivetrain.calculateProjectileSpeed(rpm);

        double hoodRad = Math.toRadians(hoodDeg);

        // Distance components
        double deltaX = robotPos.getDistance(targetPos);
        double deltaY = 0; // assume same height for now

        // Compute flight time using your drivetrain method
        double flightTime =
            drivetrain.computeFlightTime(
                projectileSpeed,
                deltaX,
                deltaY,
                hoodRad
            );

        // Field-relative robot velocity
        Translation2d fieldVelocity =
            drivetrain.getFieldVelocity();

        // Optional: add acceleration compensation
        Translation2d fieldAcceleration =
            drivetrain.getFieldAcceleration();

        // Basic lead compensation
        Translation2d leadOffset =
            fieldVelocity.times(flightTime)
                .plus(fieldAcceleration.times(0.5 * flightTime * flightTime));

        Translation2d virtualTarget =
            targetPos.minus(leadOffset);

        // Aim at compensated target
        Rotation2d desiredHeading =
            drivetrain.getRotationToFacePoint(virtualTarget);

        double omega =
            thetaController.calculate(
                robotPose.getRotation().getRadians(),
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
