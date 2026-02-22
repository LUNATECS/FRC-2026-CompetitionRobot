// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubSystem extends SubsystemBase {

  private TalonFX hopperMotor;
  
  /** Creates a new HopperRackSubSystem. */
  public HopperSubSystem() {

    hopperMotor = new TalonFX(Constants.Hopper_SubSystem_Constants.CAN_ID_HOPPER, Constants.CANBUS_Constants.CANIVORE);

    var motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    hopperMotor.getConfigurator().apply(motorOutputConfigs);
  }

  public void setSpeed(double speed){
    hopperMotor.set(speed);
  }

  public double getPosition(){
    return hopperMotor.getPosition().getValueAsDouble();
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("hopper position", hopperMotor.getPosition().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
