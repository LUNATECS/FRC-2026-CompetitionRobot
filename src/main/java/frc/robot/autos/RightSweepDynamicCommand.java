// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;


/** Add your docs here. */
public class RightSweepDynamicCommand {

    public static Command build(CommandSwerveDrivetrain drivetrain) 
        {
        GetBallLaneCommand detect = new GetBallLaneCommand(drivetrain);

    return detect.andThen(Commands.defer(() -> {

        String lane = detect.getResult();

        if (lane == null) {
            lane = "unknown";
        }

            switch (lane) {
                case "left":
                    return AutoFactory.followPath("Right Close Sweep Path");

                case "middle":
                    return AutoFactory.followPath("Right Middle Sweep Path");

                case "right":
                    return AutoFactory.followPath("Right Far Sweep Path");

                default:
                    return AutoFactory.followPath("Right Far Sweep Path");
            }
        }, Set.of(drivetrain)));
    }
}
