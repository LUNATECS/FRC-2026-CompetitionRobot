// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubSystem;
import frc.robot.subsystems.ShotCalc;
import frc.robot.subsystems.ShotConfigs;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodDynamicPointCommand extends Command {
  private final PIDController pid;
  private final HoodSubSystem hood;
  private final double distancefromTarget;

  /** Creates a new HoodDynamicPointCommand. */
  public HoodDynamicPointCommand(HoodSubSystem hood, double distanceFromTarget) {
    this.hood=hood;
    this.distancefromTarget = distanceFromTarget;
    pid = new PIDController(0.00125, 0, 0);
    pid.setTolerance(0.25);
    //COMPLETELY UNTUNED
    addRequirements(hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ShotConfigs shot = ShotCalc.getShot(distancefromTarget);
    pid.setSetpoint(shot.hoodDeg);
    
    double output = pid.calculate(hood.getHoodDegrees());
    //CLAMPING FOR NOW
    output = Math.max(-0.4, Math.min(0.4, output));

    hood.setSpeed(output);
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
