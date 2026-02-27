// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.HopperPositionCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubSystem;
import frc.robot.util.AllianceFliputil;


/** Add your docs here. */
public class RightOneCycleClimbAuto {
    double maxSpeed = 3.0;        //match AutoFactory constraints
    double maxAngularRate = 2.0;

    public static Command build(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric driveRequest, HopperSubSystem hopper) {
        double directionGoing = AllianceFliputil.getFlipBasedOnAllianceForGoingToCenter();
        double directionComing = AllianceFliputil.getFlipbasedOnAllianceForComingBack();
        Rotation2d heading = AllianceFliputil.getHeadingBasedOnAllianceForBumpRIGHT();
        
            return Commands.sequence(
                Commands.runOnce(() -> {
                try {
                    var path1 = PathPlannerPath.fromPathFile("Starting Path Right");
                    //have the make a starting path for these authos
                    drivetrain.resetPose(path1.getStartingHolonomicPose().get());
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }),

            new ParallelCommandGroup(AutoFactory.followPath("Starting Path Right"),
            new HopperPositionCommand(hopper, 52.5)),
            new DriveOverBumpCommand(drivetrain, heading, driveRequest, directionGoing),
            AutoFactory.followPath("Right Sweep 1 Path"),
            new DriveOverBumpCommand(drivetrain, heading, driveRequest, directionComing),
            //Then we would need to have another dead reckoning over bump
            AutoFactory.followPath("Right 1 Cycle To Climb Shoot"),
            AutoFactory.followPath("Right 1 Cycle Climb")
            //AutoFactory.pathFindThenFollowPath("1 Cycle Climb Shoot")
            //BECAUSE we can pathfind over the bump!!! W/ Lydia
            //then shoot or something like that
            
        );
    }
}
