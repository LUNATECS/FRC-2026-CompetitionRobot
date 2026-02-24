// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TargetSupplier {
    private static Translation2d targetPose = new Translation2d();
    private static Alliance alliance = DriverStation.getAlliance().get();
    private static Pose2d robotPose = new Pose2d();

    public TargetSupplier(CommandSwerveDrivetrain drivetrain){

    }
        public static Translation2d getTargetBasedOnAlliance(CommandSwerveDrivetrain drivetrain){
            robotPose = drivetrain.getPose();
            if(alliance == Alliance.Red){
                if(robotPose.getX() > 11.3){
                     targetPose = new Translation2d(11.91, 4.03);
                } else if(robotPose.getX() < 11.3){
                    if(robotPose.getY() < 4){
                        targetPose = new Translation2d(14.5, 2);
                    } else if(robotPose.getY() > 4){
                        targetPose = new Translation2d(14.5, 6);
                    }
                }
            } 
            if(alliance == Alliance.Blue){
                if(robotPose.getX() < 5){
                     targetPose = new Translation2d(4.625, 4.125);
                } else if(robotPose.getX() > 5){
                    if(robotPose.getY() < 4){
                        targetPose = new Translation2d(2, 2);
                    } else if(robotPose.getY() > 4){
                        targetPose = new Translation2d(2, 6);
                    }
                }
            }
            else {
                targetPose = new Translation2d(0, 0);
            }
            return targetPose;
        }



}
