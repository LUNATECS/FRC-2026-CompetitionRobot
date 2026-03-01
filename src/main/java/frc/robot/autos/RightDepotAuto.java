// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AimAndShoot.FullShootAUTOSequence;
import frc.robot.commands.HopperPositionCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubSystem;
import frc.robot.subsystems.HopperFeederSubSystem;
import frc.robot.subsystems.HopperSubSystem;
import frc.robot.subsystems.ShooterFeederSubSystem;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.util.AllianceFliputil;
import frc.robot.util.SOTMCalc;
import frc.robot.util.TargetSupplier;


/** Add your docs here. */
public class RightDepotAuto {
    double maxSpeed = 3.0;        //match AutoFactory constraints
    double maxAngularRate = 2.0;

    public static Command build(CommandSwerveDrivetrain drivetrain, 
        SwerveRequest.FieldCentric driveRequest, 
        HoodSubSystem hood, 
        ShooterSubSystem shooter, 
        HopperFeederSubSystem hopperFeeder, 
        ShooterFeederSubSystem shooterFeeder,
        HopperSubSystem hopper) 
        {
        double directionGoing = AllianceFliputil.getFlipBasedOnAllianceForGoingToCenter();
        double directionComing = AllianceFliputil.getFlipbasedOnAllianceForComingBack();
        Rotation2d headingGoing = AllianceFliputil.getHeadingBasedOnAllianceForBumpRIGHT();
        Rotation2d headingComing = AllianceFliputil.getHeadingBasedOnAllianceForBumpLEFT();
        
            return Commands.sequence(
                Commands.runOnce(() -> {
                try {
                    var path1 = PathPlannerPath.fromPathFile("Starting Path Right");
                    //have the make a starting path for these authos
                    drivetrain.resetPose(path1.getStartingHolonomicPose().get());
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }),

            new ParallelCommandGroup(AutoFactory.followPath("Starting Path Right"),
            new HopperPositionCommand(hopper, 52.5)),
            new DriveOverBumpCommand(drivetrain, headingGoing, driveRequest, directionGoing),
            AutoFactory.followPath("Right Big Sweep Path"),
            new DriveOverBumpCommand(drivetrain, headingComing, driveRequest, directionComing),
            AutoFactory.followPath("Left Shoot Path"),
            //wait for 5 seconds when we should
            new ParallelDeadlineGroup(new WaitCommand(5)
                , new FullShootAUTOSequence(drivetrain, shooter, hood, hopperFeeder, shooterFeeder, hopper, 
                    () -> TargetSupplier.getTargetBasedOnAlliance(drivetrain), driveRequest, 
                    ()->true, 
                    () -> SOTMCalc.getInstance().getParameters(
                      drivetrain.getPose().getTranslation()
                          .getDistance(TargetSupplier.getTargetBasedOnAlliance(drivetrain))).rpm(),
                          //hood angle supplier
                           () -> SOTMCalc.getInstance().getParameters(
                      drivetrain.getPose().getTranslation()
                          .getDistance(TargetSupplier.getTargetBasedOnAlliance(drivetrain))).hoodAngle())),
                    new ParallelCommandGroup(AutoFactory.followPath("Get To Depot"),
                    new HopperPositionCommand(hopper, 52.5)),
                    AutoFactory.followPath("Intake At Depot"),
                    AutoFactory.followPath("Shoot From Depot"),
                    new ParallelDeadlineGroup( new WaitCommand(3)
                        , new FullShootAUTOSequence(drivetrain, shooter, hood, hopperFeeder, shooterFeeder, hopper, 
                            () -> TargetSupplier.getTargetBasedOnAlliance(drivetrain), driveRequest, 
                            ()->true, 
                            () -> SOTMCalc.getInstance().getParameters(
                            drivetrain.getPose().getTranslation()
                                .getDistance(TargetSupplier.getTargetBasedOnAlliance(drivetrain))).rpm(),
                                //hood angle supplier
                                () -> SOTMCalc.getInstance().getParameters(
                            drivetrain.getPose().getTranslation()
                                .getDistance(TargetSupplier.getTargetBasedOnAlliance(drivetrain))).hoodAngle()))
        );
    }
}
