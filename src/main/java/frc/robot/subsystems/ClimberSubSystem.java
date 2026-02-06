// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberSubSystem extends SubsystemBase {
  
  private TalonFX climberMotor;
  
  /** Creates a new ShooterFeederSubSystem. */
  public ClimberSubSystem() {
    climberMotor = new TalonFX(Constants.Climber_SubSystem_Constants.CAN_ID_CLIMBER);

    var motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

    climberMotor.getConfigurator().apply(motorOutputConfigs);

  }

public void setSpeed(double speed){
  climberMotor.set(speed);

}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
