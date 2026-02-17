// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubSystem extends SubsystemBase {
  /** Creates a new ShooterSubSystem. */

  private TalonFX shooterMotor1;
  private TalonFX shooterMotor2;
  private TalonFX shooterMotor3;
  private TalonFX shooterMotor4;
  public double shooterTargetRPM = 0;

  private final double gearRatio = 1.25;

  private final VelocityVoltage motorVelocity = new VelocityVoltage(0);

  
  public ShooterSubSystem() {
    shooterMotor1 = new TalonFX(Constants.Shooter_SubSystem_Constants.CAN_ID_SHOOTER1 , Constants.CANBUS_Constants.CANIVORE);
    shooterMotor2 = new TalonFX(Constants.Shooter_SubSystem_Constants.CAN_ID_SHOOTER2, Constants.CANBUS_Constants.CANIVORE);
    shooterMotor3 = new TalonFX(Constants.Shooter_SubSystem_Constants.CAN_ID_SHOOTER3 , Constants.CANBUS_Constants.CANIVORE);
    shooterMotor4 = new TalonFX(Constants.Shooter_SubSystem_Constants.CAN_ID_SHOOTER4, Constants.CANBUS_Constants.CANIVORE);

    shooterMotor2.setControl(new Follower(Constants.Shooter_SubSystem_Constants.CAN_ID_SHOOTER1, MotorAlignmentValue.Aligned));
    shooterMotor3.setControl(new Follower(Constants.Shooter_SubSystem_Constants.CAN_ID_SHOOTER1, MotorAlignmentValue.Opposed));
    shooterMotor4.setControl(new Follower(Constants.Shooter_SubSystem_Constants.CAN_ID_SHOOTER1, MotorAlignmentValue.Opposed));

    var feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.SensorToMechanismRatio = gearRatio;

    var motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    var currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = 40;   //increase this current limit if necessary
    currentLimits.SupplyCurrentLimitEnable = true;

    var slot0Configs = new Slot0Configs();
    //Slot configs untested but are good base values
    slot0Configs.kV = 0.18;
    slot0Configs.kP = 0.29;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    //FOR ADJUSTING, FIRST SET PID VALUES TO 0, AND SEE IF kV is reaching setpoint consistenly across all ranges, then
    //test recovery by increasing/decreasing the P value. 

    var voltageConfigs = new VoltageConfigs();
    voltageConfigs.PeakForwardVoltage = 12;
    voltageConfigs.PeakReverseVoltage = -12;

    shooterMotor1.getConfigurator().apply(slot0Configs);
    shooterMotor1.getConfigurator().apply(feedbackConfigs);
    shooterMotor1.getConfigurator().apply(motorOutputConfigs);
    shooterMotor1.getConfigurator().apply(currentLimits);
    shooterMotor1.getConfigurator().apply(voltageConfigs);

    motorVelocity.Slot = 0;
  }

  public double getRPM(){
    return shooterMotor1.getVelocity().getValueAsDouble()*60;
  }

  public void setRPM(double rpm){
    shooterTargetRPM = rpm;
    shooterMotor1.setControl(motorVelocity.withVelocity(rpm/60));
  }

  public void setSpeed(double speed){
    shooterMotor1.set(speed);
  }

  public void setVoltage (double volts){
    shooterMotor1.setVoltage(volts);
  }


  public boolean atSpeed() {
  return Math.abs(getRPM() - shooterTargetRPM) < 50
       && shooterTargetRPM > 0;
  // TO ADD FOR LATER: Check Recovery time/Consistentcy
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Target RPM", shooterTargetRPM);
    SmartDashboard.putBoolean("Shooter At Speed", atSpeed());
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter RPM", getRPM());
    SmartDashboard.putNumber("Shooter motor voltage", shooterMotor1.getMotorVoltage().getValueAsDouble());
  }
}
