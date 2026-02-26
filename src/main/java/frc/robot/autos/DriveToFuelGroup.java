// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubSystem;

/** Auto command to drive toward detected game piece while facing it and running intake. */
public class DriveToFuelGroup {

    private static final double INTAKE_SPEED = 6000; // Intake speed in rpms

    public static Command build(CommandSwerveDrivetrain drivetrain, IntakeSubSystem intake) {
        return Commands.defer(() -> {

            Pose2d targetPose = drivetrain.getDetectedGamePiecePose(); // target fuellis
            if (targetPose == null) return Commands.none(); // do nothing if no fuels

            // heading to target from current pose (same math to point to target for aiming to shoot)
            Translation2d robotPos = drivetrain.getPose().getTranslation();
            Translation2d targetPos = targetPose.getTranslation();
            Rotation2d headingToTarget = targetPos.minus(robotPos).getAngle();

            // Create a new Pose2d with the correct heading
            Pose2d targetWithHeading = new Pose2d(targetPos, headingToTarget);

            // Drive toward the target while running the intake
            return new ParallelDeadlineGroup(
                AutoFactory.pathfindToPose(targetWithHeading),
                new IntakeCommand(intake, INTAKE_SPEED)
            );

        }, Set.of(drivetrain, intake));
    }
}