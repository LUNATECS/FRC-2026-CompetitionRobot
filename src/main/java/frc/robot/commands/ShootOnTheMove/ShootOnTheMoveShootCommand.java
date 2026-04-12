// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootOnTheMove;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.HoodDynamicPointCommand;
import frc.robot.commands.ShooterFeederDynamicCommand;
import frc.robot.subsystems.HopperFeederSubSystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.subsystems.ShooterFeederSubSystem;
import frc.robot.subsystems.ShooterSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootOnTheMoveShootCommand extends Command {
  /** Creates a new ShootCommand. */
  HopperFeederSubSystem hopperFeeder;
  ShooterFeederSubSystem shooterFeeder;
  ShooterSubSystem shooter;
  HoodDynamicPointCommand hoodCommand;
  ShootOnTheMoveCommand drivetrainCommand;
  BooleanSupplier fireButton;
  ShooterFeederDynamicCommand feederCommand;
  IntakeSubSystem intake;
  LEDSubsystem led;
  private final double currentThreshold = 400; // tune this, in amps
  private final double reverseTime = 0.2; //in seconds

  private boolean isReversing = false;
  private final Timer reverseTimer = new Timer();
  public ShootOnTheMoveShootCommand(
        HopperFeederSubSystem hopperFeeder,
        ShooterFeederSubSystem shooterFeeder,
        ShooterSubSystem shooter,
        HoodDynamicPointCommand hoodCommand,
        IntakeSubSystem intake,
        LEDSubsystem led,

        BooleanSupplier fireButton
  ) {
    this.hopperFeeder = hopperFeeder;
    this.shooterFeeder = shooterFeeder;
    this.shooter = shooter;
    this.hoodCommand = hoodCommand;
    this.fireButton = fireButton;
    this.intake = intake;
    this.led = led;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopperFeeder, led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double hopperCurrent = hopperFeeder.getFeederCurrent();
    double shooterFeederCurrent = shooterFeeder.getCurrent();

    boolean jamDetected = hopperCurrent > currentThreshold ||
                        shooterFeederCurrent > currentThreshold;
    boolean readyToShoot = shooter.atSpeed(); //&& hoodCommand.atSetpoint();
    boolean firing = fireButton.getAsBoolean() && readyToShoot;

    LEDState desiredState;

    if (firing) {
        desiredState = LEDState.SHOOTING;
    } else if (readyToShoot) {
        desiredState = LEDState.AIMED;
    } else {
        desiredState = LEDState.IDLE;
    }
    if (firing
                //&&drivetrainCommand.isAligned()
                ) {
                  if (jamDetected && !isReversing) {
                      isReversing = true;
                      reverseTimer.restart(); //restarts timer
                  }

                  // If reversing just keep reversing until timer ends
                  if (isReversing) {
                      hopperFeeder.setSpeed(-1);
                      //hopperFeeder.setTopRollerSpeed(-1);
                      shooterFeeder.setSpeed(-1);
                      intake.setSpeed(0);

                      if (reverseTimer.hasElapsed(reverseTime)) {
                          isReversing = false;
                      }

                  } else {
                      // Always feed normally when not reversing
                      hopperFeeder.setSpeed(0.95);
                      //hopperFeeder.setTopRollerSpeed(1);
                      //setting everything with rpm's now
                      intake.setRPM(2000);
                      //shooterFeeder.setRPMBasedOnShooter(shooter.getRPM());
                  }
              } else {
                hopperFeeder.setSpeed(0);
                //hopperFeeder.setTopRollerSpeed(0);
                //shooterFeeder.setSpeed(0);
                intake.setSpeed(0);
                isReversing = false;
                reverseTimer.stop();
            }
            led.setState(desiredState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
                led.setState(LEDState.IDLE);
                hopperFeeder.setSpeed(0);
                //hopperFeeder.setTopRollerSpeed(0);
                shooterFeeder.setSpeed(0);
                intake.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
