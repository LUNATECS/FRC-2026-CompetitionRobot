// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HopperPositionCommand extends Command {
  private PIDController pid;
  private HopperSubSystem hopper;
  private double setpoint;
  /** Creates a new HopperPositionCommand. */
  public HopperPositionCommand(HopperSubSystem hopper, double setpoint) {
    this.setpoint = setpoint;
    this.hopper = hopper;
    pid = new PIDController(0.03, 0, 0);
    pid.setTolerance(1);
    addRequirements(hopper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.reset();
    pid.setSetpoint(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pid.calculate(hopper.getPosition());
    //cURRENTLY CLAMPED TO LESS THAN MAX SPEED
    output = MathUtil.clamp(output, -0.5, 0.5);
    hopper.setSpeed(output);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    return pid.atSetpoint();
  }
}
