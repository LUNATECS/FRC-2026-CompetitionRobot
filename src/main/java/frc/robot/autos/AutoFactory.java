// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.io.IOException;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class AutoFactory {
        //EVERYTHING HERE NEEDS TO BE TUNED

    

    private static final PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
    
        public static Command pathFindThenFollowPath(String name) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(name);
            return AutoBuilder.pathfindThenFollowPath(path, constraints);
        } catch(Exception e) {
            e.printStackTrace();
            return Commands.none();
        }
    }


        public static Command followPath(String name) {
            try {
                PathPlannerPath path = PathPlannerPath.fromPathFile(name);
                return AutoBuilder.followPath(path);
            }
            catch(Exception e) {
                e.printStackTrace();
                return Commands.none();
            }
        }

        public static Command followPathForAlliance(String name) {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue); // default to Blue if unset
            try {
                PathPlannerPath path = PathPlannerPath.fromPathFile(name);
                // Flip/mirror automatically based on alliance
                if(alliance == Alliance.Red) {
                    path.flipPath();   // X-axis flip
                    path.mirrorPath(); // Y-axis flip if needed
                }
                return AutoBuilder.followPath(path);
            } catch(Exception e) {
                e.printStackTrace();
                return Commands.none();
            }
        }
        
}
