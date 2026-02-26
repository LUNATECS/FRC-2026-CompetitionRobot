// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class AutoFactory {



    //EVERYTING HAS TO BE TUNED

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

        private static final PathConstraints intakeBallsConstraints = new PathConstraints(
        1.5, 1.5,
        Units.degreesToRadians(540), Units.degreesToRadians(720));


        public static Command pathfindToPose(Pose2d pose) {
            return AutoBuilder.pathfindToPose(pose, intakeBallsConstraints);
        }

        
        
}
