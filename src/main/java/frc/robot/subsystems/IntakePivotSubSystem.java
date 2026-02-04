// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePivotSubSystem extends SubsystemBase {
  
  private TalonFX intakePivotMotor;

  /** Creates a new IntakePivotSubSystem. */
  public IntakePivotSubSystem() {
    intakePivotMotor = new TalonFX(Constants.Intake_Pivot_SubSystem_Constants.CAN_ID_INTAKE_PIVOT);

    var motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

    intakePivotMotor.getConfigurator().apply(motorOutputConfigs);

  }

  public void setSpeed(double speed){
    intakePivotMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
