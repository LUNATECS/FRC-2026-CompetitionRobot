// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;


/** Add your docs here. */
public class PracticeAuto {

    public static Command build() {


        double maxSpeed = 3.0;        //match AutoFactory constraints
        double maxAngularRate = 2.0;

        return Commands.sequence(
            AutoFactory.followPath("Example Cycle Path"),
            AutoFactory.followPathForAlliance("Climb Path")
        );
    }
}
