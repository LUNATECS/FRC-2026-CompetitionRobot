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
        else if (distanceMeters < 2 && distanceMeters > 1.25){
            return new ShotConfigs(7.5, interpolate(distanceMeters, 1.25, 2, 800, 1000));
        }

        else if (distanceMeters < 3 && distanceMeters >2){
            return new ShotConfigs(15, interpolate(distanceMeters, 2, 3, 1000, 1100));
        }

        else if (distanceMeters < 4 && distanceMeters >3){
            return new ShotConfigs(15, interpolate(distanceMeters, 3, 4, 1100, 1250));
        }

        else if (distanceMeters < 4.7 && distanceMeters > 4) {
        return new ShotConfigs(
            15.0,interpolate(distanceMeters, 4.0, 4.7, 1200, 1400)
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
