// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperFeederSubSystem extends SubsystemBase {
  /** Creates a new HopperFeederSubSystem. */

  private TalonFX hopperFeederMotor;
  private TalonFX topFeederMotor;
  private final VelocityVoltage hopperMotorVelocity = new VelocityVoltage(0);
  private final VelocityVoltage topFeederMotorVelocity = new VelocityVoltage(0);
  private double hopperDiameterWheels;
  private double topDiameterWheels;


  public HopperFeederSubSystem() {

    hopperFeederMotor = new TalonFX(Constants.Hopper_Feeder_SubSystem_Constants.CAN_ID_FEEDER_BELTS, Constants.CANBUS_Constants.CANIVORE);
    topFeederMotor = new TalonFX(Constants.Hopper_Feeder_SubSystem_Constants.CAN_ID_TOP_FEEDER_MOTOR, Constants.CANBUS_Constants.CANIVORE);

    var hopperMotorOutputConfigs = new MotorOutputConfigs();
    hopperMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    hopperMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    hopperFeederMotor.getConfigurator().apply(hopperMotorOutputConfigs);

    var topFeederMotorOutputConfigs = new MotorOutputConfigs();
    topFeederMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    topFeederMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

    topFeederMotor.getConfigurator().apply(topFeederMotorOutputConfigs);

    var slot0Configs = new Slot0Configs();
    //Slot configs untested but are good base values
    slot0Configs.kV = 0.2;
    slot0Configs.kP = 0.4;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    hopperFeederMotor.getConfigurator().apply(slot0Configs);
    hopperMotorVelocity.Slot = 0;
    var slot1Configs = new Slot1Configs();
   slot1Configs.kV = 0.2;
    slot1Configs.kP = 0.4;
    slot1Configs.kI = 0;
    slot1Configs.kD = 0;
    topFeederMotor.getConfigurator().apply(slot1Configs);
    topFeederMotorVelocity.Slot = 1;
  }

  public void setRPMHopper(double rpm){
    hopperFeederMotor.setControl(hopperMotorVelocity.withVelocity(rpm));
}
  public void setRPMtopFeeder(double rpm){
    topFeederMotor.setControl(topFeederMotorVelocity.withVelocity(rpm));
  }
   public void setSpeed(double speed){
    hopperFeederMotor.set(speed);
  }

  public void setTopRollerSpeed(double speed){
    topFeederMotor.set(speed);
  }
  //get current for motor stalling

  public double getTopFeederCurrent() {
    return topFeederMotor.getStatorCurrent().getValueAsDouble();
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hopper Feeder RPM", hopperFeederMotor.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putNumber("Top Feeder AMPS", getTopFeederCurrent());
    // This method will be called once per scheduler run
  }
}
