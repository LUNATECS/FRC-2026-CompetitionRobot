// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TrenchPositionSupplier {
    
        public TrenchPositionSupplier(){
        }

        public static boolean getTrenchZone(CommandSwerveDrivetrain drivetrain){
            
            Pose2d robotPose = new Pose2d();
            boolean trenchZone = false;
            robotPose = drivetrain.getPose();
           
                if(robotPose.getX() < 5.25 && robotPose.getX() > 4){
                      if(robotPose.getY() > 7 || robotPose.getY() < 1.1){
                          trenchZone = true;
                      }
                }

                if(robotPose.getX() < 12.625 && robotPose.getX() > 11.25){
                      if(robotPose.getY() > 7 || robotPose.getY() < 1.1){
                          trenchZone = true;
                      }
                }
            return trenchZone;
        }

}
