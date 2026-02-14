// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.InvertedValue;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubSystem extends SubsystemBase {
  
  private TalonFX intakeMotor;

  /** Creates a new IntakeSubSystem. */ 
  public IntakeSubSystem() {
    intakeMotor = new TalonFX(Constants.Intake_SubSystem_Constants.CAN_ID_INTAKE, Constants.CANBUS_Constants.CANIVORE);

    var motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = 0;
    slot0Configs.kP = 0;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    intakeMotor.getConfigurator().apply(motorOutputConfigs);
    intakeMotor.getConfigurator().apply(slot0Configs);

  }

  public double getRPM(){
    return intakeMotor.getVelocity().getValueAsDouble()*60;
  }

  public void setRPM(double rpm){
    intakeMotor.set(rpm);
  }

  public void setSpeed(double speed){
    intakeMotor.set(speed);
  }

  public void setVoltage(double volts){
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake rpm", getRPM());
  }
}
