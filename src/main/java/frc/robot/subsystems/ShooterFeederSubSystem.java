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

public class ShooterFeederSubSystem extends SubsystemBase {
  
  private TalonFX shooterFeeder;
  
  /** Creates a new ShooterFeederSubSystem. */
    private final VelocityVoltage motorVelocity = new VelocityVoltage(0);

    private double feederMotortoShooterMotorRatio = 2.96296;

  public ShooterFeederSubSystem() {
    shooterFeeder = new TalonFX(Constants.Shooter_Feeder_SubSystem_Constants.CAN_ID_SHOOTER_FEEDER, Constants.CANBUS_Constants.CANIVORE);

    var motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

    shooterFeeder.getConfigurator().apply(motorOutputConfigs);

    var slot0Configs = new Slot0Configs();
    //Slot configs untested but are good base values
    slot0Configs.kV = 0.18;
    slot0Configs.kP = 0.29;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    shooterFeeder.getConfigurator().apply(slot0Configs);
    motorVelocity.Slot = 0;
  }

public void setSpeed(double speed){
  shooterFeeder.set(speed);
  //spins counterclockwise
}

public void setRPMBasedOnShooter(double Shooterrpm){
    double rpmspeed = Shooterrpm*feederMotortoShooterMotorRatio;
    shooterFeeder.setControl(motorVelocity.withVelocity(rpmspeed/60));
}
//get current for stalling/spikes
public double getCurrent() {
  return shooterFeeder.getStatorCurrent().getValueAsDouble();
}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Feeder RPM", shooterFeeder.getVelocity().getValueAsDouble()/60);
    SmartDashboard.putNumber("Shooter Feeder AMPS", getCurrent());
    // This method will be called once per scheduler run
  }
}
