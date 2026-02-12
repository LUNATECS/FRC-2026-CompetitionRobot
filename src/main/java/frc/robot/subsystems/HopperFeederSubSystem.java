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

public class HopperFeederSubSystem extends SubsystemBase {
  /** Creates a new HopperFeederSubSystem. */

  private TalonFX hopperFeederMotor;

  public HopperFeederSubSystem() {

    hopperFeederMotor = new TalonFX(Constants.Hopper_Feeder_SubSystem_Constants.CAN_ID_FEEDER_BELTS, "canivore");

    var motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    hopperFeederMotor.getConfigurator().apply(motorOutputConfigs);
  }

   public void setSpeed(double speed){
    hopperFeederMotor.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
