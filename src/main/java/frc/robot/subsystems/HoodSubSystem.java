// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubSystem extends SubsystemBase {
  /** Creates a new HoodSubSystem. */

  private TalonFX hoodMotor;
  private CANcoder hoodEncoder;

  private final double gearRatio = 23.125;
  private final double ticksToDegrees = 1;
  private final double offset = 0;

  public HoodSubSystem() {
    hoodMotor = new TalonFX(Constants.Hood_SubSystem_Constants.CAN_ID_HOOD, Constants.CANBUS_Constants.CANIVORE);
    //It is a throughbore encoder, we just have to make sure it is still a CANCoder
    hoodEncoder = new CANcoder(Constants.Hood_SubSystem_Constants.CAN_ID_HOOD_ENCODER);

    var motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    var feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.SensorToMechanismRatio = gearRatio;

    hoodMotor.getConfigurator().apply(motorOutputConfigs);
    hoodMotor.getConfigurator().apply(feedbackConfigs);
  }

  public double getHoodDegrees(){
    return (hoodEncoder.getAbsolutePosition().getValueAsDouble() - offset)*ticksToDegrees;
  }

  public void setSpeed(double speed){
    hoodMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
