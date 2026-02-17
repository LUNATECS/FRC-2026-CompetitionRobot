// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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
  private final VelocityVoltage motorVelocity = new VelocityVoltage(0);


  public HopperFeederSubSystem() {

    hopperFeederMotor = new TalonFX(Constants.Hopper_Feeder_SubSystem_Constants.CAN_ID_FEEDER_BELTS, Constants.CANBUS_Constants.CANIVORE);

    var motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    hopperFeederMotor.getConfigurator().apply(motorOutputConfigs);

    var slot0Configs = new Slot0Configs();
    //Slot configs untested but are good base values
    slot0Configs.kV = 0.2;
    slot0Configs.kP = 0.4;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    hopperFeederMotor.getConfigurator().apply(slot0Configs);
    motorVelocity.Slot = 0;
  }

  public void setRPM(double rpm){
    hopperFeederMotor.setControl(motorVelocity.withVelocity(rpm));
}

   public void setSpeed(double speed){
    hopperFeederMotor.set(speed);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hoper Feeder RPM", hopperFeederMotor.getVelocity().getValueAsDouble()*60);
    // This method will be called once per scheduler run
  }
}
