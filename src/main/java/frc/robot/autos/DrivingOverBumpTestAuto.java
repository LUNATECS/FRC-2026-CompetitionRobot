// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.AllianceFliputil;

/** Add your docs here. */
public class DrivingOverBumpTestAuto {
    public static Command build(CommandSwerveDrivetrain drivetrain, 
        SwerveRequest.FieldCentric driveRequest) 
        {
            return Commands.defer(()->{
            double directionGoing = AllianceFliputil.getFlipBasedOnAllianceForGoingToCenter();
            double directionComing = AllianceFliputil.getFlipbasedOnAllianceForComingBack();
            Rotation2d heading = AllianceFliputil.getHeadingBasedOnAllianceForBumpLEFT();
        
        
            return Commands.sequence(
                new DriveOverBumpCommand(drivetrain, heading, driveRequest, directionGoing), 
                new DriveOverBumpCommand(drivetrain, heading, driveRequest, directionComing) 
            );
        }, Set.of(drivetrain));

    }
}
