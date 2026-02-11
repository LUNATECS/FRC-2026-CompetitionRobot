// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivotSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakePivotCommand extends Command {

  private PIDController pController;
  private IntakePivotSubSystem intakePivotSubSystem;

  /** Creates a new IntakePivotCommand. */
  public IntakePivotCommand(IntakePivotSubSystem intakePivotSubSystem, double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakePivotSubSystem = intakePivotSubSystem;
    addRequirements(intakePivotSubSystem);

    pController = new PIDController(0.0084, 0, 0);
    pController.setSetpoint(degrees);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pController.reset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pController.calculate(intakePivotSubSystem.getDegreesOfPivot());
    if(speed >= 0.5){
      intakePivotSubSystem.setSpeed(0.5);
    } 
    if(speed <= -0.5){
      intakePivotSubSystem.setSpeed(-0.5);
    }
    if(speed < 0.5 && speed > -0.5){
      intakePivotSubSystem.setSpeed(speed);
    }

    //intakePivotSubSystem.setSpeed(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakePivotSubSystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return pController.atSetpoint();
  }
}
