// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubSystem;
import frc.robot.subsystems.ShooterSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HopperAutoAgitateCommand extends Command {
  private final HopperSubSystem hopper;
  private final Timer timer = new Timer();

  // Tune these
  private final double inwardSpeed = -0.425;   // push balls inward
  private final double outwardSpeed = 0.2;   // slight retract
  private final double inwardTime = 0.6;      // seconds
  private final double outwardTime = 0.25;     // seconds

  private final double stopThreshold = 5.0; //hopper position encoder value thing
  private final BooleanSupplier fireButton;
  private final ShooterSubSystem shooter;
  private final HoodDynamicPointCommand hoodCommand;
  /** Creates a new HopperAgitateTrigerredCommand. */
  public HopperAutoAgitateCommand(HopperSubSystem hopper,
    BooleanSupplier fireButton,
    ShooterSubSystem shooter,
    HoodDynamicPointCommand hoodCommand) {
    this.hopper = hopper;
    this.fireButton = fireButton;
    this.shooter = shooter;
    this.hoodCommand = hoodCommand;
    addRequirements(hopper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(fireButton.getAsBoolean() &&
                  shooter.atSpeed() &&
                  hoodCommand.atSetpoint()){
          double position = hopper.getPosition();

          // If near zero, stop moving
          if (position <= stopThreshold) {
            hopper.setSpeed(0);
            return;
          }

          double cycleTime = inwardTime + outwardTime;
          double timeInCycle = timer.get() % cycleTime;

          if (timeInCycle < inwardTime) {
            hopper.setSpeed(inwardSpeed);
          } else {
            hopper.setSpeed(outwardSpeed);
          }
    } else {
      hopper.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return hopper.getPosition() <= stopThreshold;
    return false; // run until interrupted
  }
}
