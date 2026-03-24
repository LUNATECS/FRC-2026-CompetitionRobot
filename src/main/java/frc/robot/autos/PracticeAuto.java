// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Add your docs here. */
public class PracticeAuto {

    public static Command build(CommandSwerveDrivetrain drivetrain) {
        return Commands.sequence(
            Commands.runOnce(() -> {
            try {
                drivetrain.resetPose(drivetrain.getPose());
            } catch (Exception e) {
                e.printStackTrace();
            }
        })
            //AutoFactory.followPath("StraightlineTune")
        );
    }
}
