// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HopperAgitateCommand extends Command {
  private final HopperSubSystem hopper;
  private final Timer timer = new Timer();

  // Tune these
  private final double inwardSpeed = -0.5;   // push balls inward
  private final double outwardSpeed = 0.15;   // slight retract
  private final double inwardTime = 0.7;      // seconds
  private final double outwardTime = 0.4;     // seconds

  private final double stopThreshold = 5.0; //hopper position encoder value thing
  /** Creates a new HopperAgitateCommand. */
  public HopperAgitateCommand(HopperSubSystem hopper) {
    this.hopper = hopper;
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
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hopper.getPosition() <= stopThreshold;
  }
}
