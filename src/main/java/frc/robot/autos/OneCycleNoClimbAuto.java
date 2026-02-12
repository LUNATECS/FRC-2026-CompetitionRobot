// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.AutoFactory.AutoList;

/** Add your docs here. */
public class OneCycleNoClimbAuto {
    double maxSpeed = 3.0;        //match AutoFactory constraints
    double maxAngularRate = 2.0;

    public static Command build() {
        return Commands.sequence(

            //We first HAVE to reset our odometry manually depending on the auto
            AutoFactory.resetPoseForAuto(AutoList.ONE_CYCLE_NO_CLIMB),
            //This auto would start with a shoot commmand
            //Then it would go into a dead reckoning over the bump at the beggining
            AutoFactory.followPathForAlliance("1 Cycle Climb Collect"),
            AutoFactory.followPathForAlliance("1 Cycle Climb Get Ball"),
            AutoFactory.followPathForAlliance("1 Cycle Climb Return"),
            //Then we would need to have another dead reckoning over bump
            AutoFactory.followPathForAlliance("1 Cycle Climb Shoot"),
            //OR
            AutoFactory.pathFindThenFollowPathForAlliance("1 Cycle Climb Shoot")
            //BECAUSE we can pathfind over the bump!!! W/ Lydia
            //then shoot or something like that
            
        );
    }
}
