// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubSystem extends SubsystemBase {
  TalonFX hopperMotor;

  /** Creates a new HopperRackSubSystem. */
  public HopperSubSystem() {
    hopperMotor = new TalonFX(Constants.Hopper_Feeder_SubSystem_Constants.CAN_ID_FEEDER_BELTS);
  }

  public void setSpeed(double speed){
    hopperMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
