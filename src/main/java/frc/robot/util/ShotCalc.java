// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import javax.xml.validation.SchemaFactoryConfigurationError;

/** Add your docs here. */
public class ShotCalc {
    private ShotCalc() {}


    public static ShotConfigs getShot(double distanceMeters) {
        //THE X value REPRESENTS the distance from the target.
        //We need to make a table between different distance values, and the RPM/Hood Degrees
        //The goal of this is to keep the same hood degrees for set intervals, while interpolating the RPM.
        if (distanceMeters < 1.25){
            return new ShotConfigs(0, 1000);
        }
        else if (distanceMeters < 1.75 && distanceMeters > 1.25){
            return new ShotConfigs(7.5, interpolate(distanceMeters, 1.75, 2, 950, 1000));
        }

        else if (distanceMeters < 2.5 && distanceMeters >2){
            return new ShotConfigs(7.5, interpolate(distanceMeters, 2, 2.5, 1000, 1100));
        }

        else if (distanceMeters < 3.5 && distanceMeters >3){
            return new ShotConfigs(15, interpolate(distanceMeters, 3, 3.5, 1090, 1150));
        }

        else if (distanceMeters < 4.5 && distanceMeters > 4){
            return new ShotConfigs(20, interpolate(distanceMeters, 4, 4.5, 1215, 1300));
        }

        else if (distanceMeters < 5 && distanceMeters > 4) {
        return new ShotConfigs(
            20,interpolate(distanceMeters, 4.0, 4.5, 1215, 1300)
        );
        } else {
            return new ShotConfigs(30, 4000);
        }
    }

    public static double interpolate(
        double x, double x1, double x2, double y1, double y2) {

        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }
}
