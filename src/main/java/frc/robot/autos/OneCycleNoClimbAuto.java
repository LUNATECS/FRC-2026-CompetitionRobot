// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Add your docs here. */
public class OneCycleNoClimbAuto {
    double maxSpeed = 3.0;        //match AutoFactory constraints
    double maxAngularRate = 2.0;
    RobotContainer robotcontainer;

    public static Command build(CommandSwerveDrivetrain drivetrain) {
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

            //We first HAVE to reset our odometry manually depending on the auto, or we
            //might not even have to if we have good vision at the beginning
            //This auto would start with a shoot commmand
            //Then it would go into a dead reckoning over the bump at the beggining
            //new DriveOverBumpCommand(drivetrain, -45, , 1),
            AutoFactory.followPath("1 Cycle Climb Collect"),
            AutoFactory.followPath("1 Cycle Climb Get Ball"),
            AutoFactory.followPath("1 Cycle Climb Return"),
            //Then we would need to have another dead reckoning over bump
            AutoFactory.followPath("1 Cycle Climb Shoot"),
            //OR
            AutoFactory.pathFindThenFollowPath("1 Cycle Climb Shoot")
            //BECAUSE we can pathfind over the bump!!! W/ Lydia
            //then shoot or something like that
            
        );
    }
}
