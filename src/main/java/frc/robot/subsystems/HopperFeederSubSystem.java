// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperFeederSubSystem extends SubsystemBase {
  /** Creates a new HopperFeederSubSystem. */

  private TalonFX feederMotor1;
  private TalonFX feederMotor2;
  private final VelocityVoltage hopperMotorVelocity = new VelocityVoltage(0);



  public HopperFeederSubSystem() {
    feederMotor1 = new TalonFX(Constants.Hopper_Feeder_SubSystem_Constants.CAN_ID_FEEDER1, Constants.CANBUS_Constants.CANIVORE);
    feederMotor2 = new TalonFX(Constants.Hopper_Feeder_SubSystem_Constants.CAN_ID_FEEDER2, Constants.CANBUS_Constants.CANIVORE);

    var hopperMotorOutputConfigs = new MotorOutputConfigs();
    hopperMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    hopperMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    feederMotor1.getConfigurator().apply(hopperMotorOutputConfigs);

    var currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = Constants.Current_Limit_Constants.CurrentLimit;   //increase this current limit if necessary (increased from 40)
    currentLimits.SupplyCurrentLimitEnable = true;

    feederMotor1.getConfigurator().apply(currentLimits);
    feederMotor2.getConfigurator().apply(currentLimits);

    var slot0Configs = new Slot0Configs();
    //Slot configs untested but are good base values
    slot0Configs.kV = 0.08;
    slot0Configs.kP = 0.006;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.015;
    feederMotor1.getConfigurator().apply(slot0Configs);
    hopperMotorVelocity.Slot = 0;

    feederMotor2.setControl(new Follower(Constants.Hopper_Feeder_SubSystem_Constants.CAN_ID_FEEDER1, MotorAlignmentValue.Opposed));



  }

  public void setRPMHopper(double rpm){
    feederMotor1.setControl(hopperMotorVelocity.withVelocity(rpm));
}
   public void setSpeed(double speed){
    feederMotor1.set(speed);
  }

  public double getFeederCurrent(){
    return feederMotor1.getStatorCurrent().getValueAsDouble();
  }
  //get current for motor stalling

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
