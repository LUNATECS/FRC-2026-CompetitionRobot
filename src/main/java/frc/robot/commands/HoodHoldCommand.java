// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodHoldCommand extends Command {
  private final PIDController pid;
  private final HoodSubSystem hood;
  private double setpoint;
    private DoubleSupplier setpointSupplier;

  public HoodHoldCommand(HoodSubSystem hood, DoubleSupplier setpointSupplier) {
    this.hood = hood;
    this.setpointSupplier = setpointSupplier;
    pid = new PIDController(0.125, 0, 0); //0.2125
    pid.setTolerance(0.25);
    SmartDashboard.putNumber("INPUT hood setpoint", 0);
    addRequirements(hood);
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
    double inputSetpoint = setpointSupplier.getAsDouble();
    pid.setSetpoint(inputSetpoint);
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
