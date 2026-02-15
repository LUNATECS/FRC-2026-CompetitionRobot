// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperFeederSubSystem;
import frc.robot.subsystems.ShooterFeederSubSystem;
import frc.robot.subsystems.ShooterSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */
  HopperFeederSubSystem hopperFeeder;
  ShooterFeederSubSystem shooterFeeder;
  ShooterSubSystem shooter;
  HoodDynamicPointCommand hoodCommand;
  FaceFieldPointCommand drivetrainCommand;
  BooleanSupplier fireButton;
  public ShootCommand(
        HopperFeederSubSystem hopperFeeder,
        ShooterFeederSubSystem shooterFeeder,
        ShooterSubSystem shooter,
        HoodDynamicPointCommand hoodCommand,
        FaceFieldPointCommand drivetrainCommand,
        BooleanSupplier fireButton
  ) {
    this.hopperFeeder = hopperFeeder;
    this.shooterFeeder = shooterFeeder;
    this.shooter = shooter;
    this.hoodCommand = hoodCommand;
    this.drivetrainCommand = drivetrainCommand;
    this.fireButton = fireButton;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterFeeder, hopperFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (fireButton.getAsBoolean() &&
                shooter.atSpeed() &&
                hoodCommand.atSetpoint() &&
                drivetrainCommand.isAligned()) {

                hopperFeeder.setSpeed(1);
                shooterFeeder.setSpeed(1);
            } else {
                hopperFeeder.setSpeed(0);
                shooterFeeder.setSpeed(0);
            }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
