// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubSystem;

public class DriveToFuelLoopAuto {

    private static final double AUTO_TIMEOUT = 10.0; // max auto time

    public static Command build(CommandSwerveDrivetrain drivetrain, IntakeSubSystem intake) {

        // repeat driving to fuel until timer expires
        return Commands.sequence(
            drivetrain.setHopperToNeural,
            new ParallelDeadlineGroup(
                new WaitCommand(AUTO_TIMEOUT)
                ,Commands.repeatingSequence(DriveToFuelGroup.build(drivetrain, intake))),
            new InstantCommand(()->{intake.setSpeed(0);}, intake),
            drivetrain.setHopperToApriltag
        ).finallyDo(interrupted -> drivetrain.setHopperLimelightPipeline(0));
        //finally do in there just in case we get interrupted
    }
}