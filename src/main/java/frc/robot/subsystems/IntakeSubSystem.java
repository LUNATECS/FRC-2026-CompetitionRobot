// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubSystem extends SubsystemBase {
  
  private TalonFX intakeMotor;
    private final VelocityVoltage motorVelocity = new VelocityVoltage(0);

  /** Creates a new IntakeSubSystem. */ 
  public IntakeSubSystem() {
    intakeMotor = new TalonFX(Constants.Intake_SubSystem_Constants.CAN_ID_INTAKE, Constants.CANBUS_Constants.CANIVORE);

    var motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = 0.05;
    slot0Configs.kP = 0.18;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    var currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = Constants.Current_Limit_Constants.CurrentLimit;   //increase this current limit if necessary (increased from 40)
    currentLimits.SupplyCurrentLimitEnable = true;

    intakeMotor.getConfigurator().apply(motorOutputConfigs);
    intakeMotor.getConfigurator().apply(slot0Configs);
    intakeMotor.getConfigurator().apply(currentLimits);
    motorVelocity.Slot = 0;

  }

  public double getRPM(){
    return intakeMotor.getVelocity().getValueAsDouble()*60;
  }

  public void setRPM(double rpm){
    intakeMotor.setControl(motorVelocity.withVelocity(rpm/60));
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
    if (Constants.Current_Limit_Constants.PlotCurrentLimits){
      SmartDashboard.putNumber("intake current", intakeMotor.getStatorCurrent().getValueAsDouble());
    }
  }
}
