// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;
//THIS HELPER CLASS IS GOING TO BE USED TO CALCULATE OUR FLYWHEEL SPEED/HOOD ANGLE DURING THE DIFFERENT SHOTS

/** Add your docs here. */
public class ShotConfigs {

public final double hoodDeg;
public final double rpm;

    public ShotConfigs(double hoodDeg, double rpm) {
        this.hoodDeg = hoodDeg;
        this.rpm = rpm;
    }
    public double getRPM() {
        return rpm;
    }
}
