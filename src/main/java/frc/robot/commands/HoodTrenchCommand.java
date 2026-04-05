// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubSystem;
import frc.robot.util.TrenchPositionSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodTrenchCommand extends Command {
  private final HoodSubSystem hood;
  private final PIDController pid;
  private final CommandSwerveDrivetrain drivetrain;
  private double LastKnownGoodSetpoint;
  /** Creates a new HoodTrenchCommand. */
  public HoodTrenchCommand(HoodSubSystem hood, CommandSwerveDrivetrain drivetrain) {
    this.hood = hood;
    this.drivetrain = drivetrain;
    pid = new PIDController(0.115, 0, 0);
    pid.setTolerance(0.5);
    addRequirements(hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LastKnownGoodSetpoint = 10;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double setpoint = 2;


    if(TrenchPositionSupplier.getTrenchZone(drivetrain)){
      setpoint = 2;
    }
    else {
      setpoint = LastKnownGoodSetpoint;
    }
    pid.setSetpoint(setpoint);
    double output = pid.calculate(hood.getHoodDegrees());
    //CLAMPING FOR NOW
    output = Math.max(-0.8, Math.min(0.8, output));

    hood.setSpeed(output);

  }

   public boolean atSetpoint() {
        return pid.atSetpoint();
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
