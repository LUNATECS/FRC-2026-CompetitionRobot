// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class AllianceFliputil {
    public static double direction = 0;
    
    public static double getFlipBasedOnAllianceForGoingToCenter() {
        DriverStation.Alliance alliance =
    DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        if (alliance == DriverStation.Alliance.Red) {
            direction = -1;
        } else if (alliance == DriverStation.Alliance.Blue){
            direction = 1;
        }
        SmartDashboard.putNumber("Direction in Auto", direction);
        return direction;
    }

    public static double getFlipbasedOnAllianceForComingBack() {
        DriverStation.Alliance alliance =
    DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        if (alliance == DriverStation.Alliance.Red) {
            direction = 1;
        } else if (alliance == DriverStation.Alliance.Blue){
            direction = -1;
        }
        return direction;
    }

    public static Rotation2d getHeadingBasedOnAllianceForBumpRIGHT() {
        DriverStation.Alliance alliance =
    DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        if (alliance == DriverStation.Alliance.Red) {
            return Rotation2d.fromDegrees(135);
        } else if (alliance == DriverStation.Alliance.Blue){
            return Rotation2d.fromDegrees(-45);
        }
        else {
            return Rotation2d.fromDegrees(0);
        }
    }

    public static Rotation2d getHeadingBasedOnAllianceForBumpLEFT() {
        DriverStation.Alliance alliance =
    DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        if (alliance == DriverStation.Alliance.Red) {
            return Rotation2d.fromDegrees(225);
        } else if (alliance == DriverStation.Alliance.Blue) {
            return Rotation2d.fromDegrees(45);
        }
        else {
            return Rotation2d.fromDegrees(0);
        }
    }
}
