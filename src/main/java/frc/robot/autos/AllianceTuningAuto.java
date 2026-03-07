// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;


/** Add your docs here. */
public class AllianceTuningAuto {
    double maxSpeed = 3.0;        //match AutoFactory constraints
    double maxAngularRate = 2.0;

    public static Command build(CommandSwerveDrivetrain drivetrain, 
        SwerveRequest.FieldCentric driveRequest) 
        {
        
        
            return Commands.sequence(
            AutoFactory.followPath("AllianceTune")
        );
    }
}
