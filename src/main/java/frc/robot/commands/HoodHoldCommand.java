// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodHoldCommand extends Command {
  private final PIDController pid;
  private final HoodSubSystem hood;
  private double setpoint;
  public HoodHoldCommand(HoodSubSystem hood, double setpoint) {
    this.hood = hood;
    this.setpoint = setpoint;
    pid = new PIDController(0.00125, 0, 0);
    pid.setSetpoint(setpoint);
    pid.setTolerance(0.5);
    //SETPOINT should be in degrees
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pid.calculate(hood.getHoodDegrees());
    //cURRENTLY CLAMPED TO LESS THAN MAX SPEED
    output = MathUtil.clamp(output, -0.5, 0.5);
    hood.setSpeed(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
