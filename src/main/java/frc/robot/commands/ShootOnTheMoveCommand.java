// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubSystem;
import frc.robot.subsystems.ShooterSubSystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootOnTheMoveCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Supplier<Translation2d> translationSupplier;
  private final Supplier<Translation2d> targetSupplier;
  private final SwerveRequest.FieldCentric driveRequest;
  private final PIDController thetaController;
  private final ShooterSubSystem shooter;
  private final HoodSubSystem hood;
  /** Creates a new ShootOnTheMoveCommand. */
  public ShootOnTheMoveCommand(
        CommandSwerveDrivetrain drivetrain,
        Supplier<Translation2d> translationSupplier,
        Supplier<Translation2d> targetSupplier,
        SwerveRequest.FieldCentric driveRequest,
        ShooterSubSystem shooter,
        HoodSubSystem hood
  ) {
        this.drivetrain = drivetrain;
        this.translationSupplier = translationSupplier;
        this.targetSupplier = targetSupplier;
        this.driveRequest = driveRequest;
        this.shooter = shooter;
        this.hood = hood;
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
    Translation2d robotPos = drivetrain.getPose().getTranslation();
    Translation2d targetPos = targetSupplier.get();
    Translation2d targetDelta = targetPos.minus(robotPos);
    double deltaY = 62 - 23.25; // in meters, hubheight minus shooter height
    double deltaX = targetDelta.getNorm(); // horizontal distance (meters)
    double hoodAngleRad = Math.toRadians(hood.getHoodDegrees());


    Translation2d robotVel = drivetrain.getFieldVelocity();
    Translation2d robotAccel = drivetrain.getFieldAcceleration();

    double projectileSpeed = drivetrain.calculateProjectileSpeed(shooter.getRPM()); // TODO: replace with real shooter model
    double tFlight = drivetrain.computeFlightTime(projectileSpeed, deltaX, deltaY, hoodAngleRad);

    Translation2d displacement =
        robotVel.times(tFlight)
            .plus(robotAccel.times(0.5 * tFlight * tFlight));

    Translation2d predictedTarget =
        targetPos.minus(displacement);

    Rotation2d desiredHeading =
        predictedTarget.minus(robotPos).getAngle();

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

    SmartDashboard.putNumber("Predicted Target X", predictedTarget.getX());
    SmartDashboard.putNumber("Predicted Target Y", predictedTarget.getY());
    SmartDashboard.putNumber("tFlight", tFlight);
    SmartDashboard.putNumber("Projectile Speed", projectileSpeed);
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
