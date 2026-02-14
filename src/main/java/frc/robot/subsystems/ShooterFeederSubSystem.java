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

public class ShooterFeederSubSystem extends SubsystemBase {
  
  private TalonFX shooterFeeder;
  
  /** Creates a new ShooterFeederSubSystem. */
  public ShooterFeederSubSystem() {
    shooterFeeder = new TalonFX(Constants.Shooter_Feeder_SubSystem_Constants.CAN_ID_SHOOTER_FEEDER, Constants.CANBUS_Constants.CANIVORE);

    var motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    shooterFeeder.getConfigurator().apply(motorOutputConfigs);

  }

public void setSpeed(double speed){
  shooterFeeder.set(speed);
  //spins counterclockwise
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
