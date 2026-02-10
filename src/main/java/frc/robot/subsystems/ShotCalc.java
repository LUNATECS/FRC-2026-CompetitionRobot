// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public class ShotCalc {
    private ShotCalc() {}


    public static ShotConfigs getShot(double distanceMeters) {
        //THE X value REPRESENTS the distance from the target.
        //We need to make a table between different distance values, and the RPM/Hood Degrees
        //The goal of this is to keep the same hood degrees for set intervals, while interpolating the RPM.
        if (distanceMeters < 4.0) {
        return new ShotConfigs(
            18.0,interpolate(distanceMeters, 2.0, 4.0, 3200, 4000)
        );
        } 
        else {
        return new ShotConfigs(
            26.0,
            interpolate(distanceMeters, 4.0, 8.0, 4600, 5000)
        );
        }
        //ONCE AGAin, THIS IS JUST A REFERENCE TABLE FOR NOW
    }
    public static double interpolate(
        double x, double x1, double x2, double y1, double y2) {

        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }
}
