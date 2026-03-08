// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AimAndShoot.FullShootAUTOSequence;
import frc.robot.commands.HopperPositionCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubSystem;
import frc.robot.subsystems.HopperFeederSubSystem;
import frc.robot.subsystems.HopperSubSystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.ShooterFeederSubSystem;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.util.AllianceFliputil;
import frc.robot.util.SOTMCalc;
import frc.robot.util.TargetSupplier;


/** Add your docs here. */
public class LeftTwoCycleSTEALAuto {
    double maxSpeed = 3.0;        //match AutoFactory constraints
    double maxAngularRate = 2.0;

    public static Command build(CommandSwerveDrivetrain drivetrain, 
        SwerveRequest.FieldCentric driveRequest, 
        HoodSubSystem hood, 
        ShooterSubSystem shooter, 
        HopperFeederSubSystem hopperFeeder, 
        ShooterFeederSubSystem shooterFeeder,
        HopperSubSystem hopper,
        IntakeSubSystem intake) 
        {
        return Commands.defer(()->{
        double directionGoing = AllianceFliputil.getFlipBasedOnAllianceForGoingToCenter();
        double directionComing = AllianceFliputil.getFlipbasedOnAllianceForComingBack();
        Rotation2d heading = AllianceFliputil.getHeadingBasedOnAllianceForBumpLEFT();
        
            return Commands.sequence(
            //     Commands.runOnce(() -> {
            //     try {
            //         var path1 = PathPlannerPath.fromPathFile("Starting Path Left");
            //         //have the make a starting path for these authos
            //         drivetrain.resetPose(path1.getStartingHolonomicPose().get());
            //     } catch (Exception e) {
            //         e.printStackTrace();
            //     }
            // }),

            new ParallelDeadlineGroup(new DriveOverBumpCommand(drivetrain, heading, driveRequest, directionGoing), 
            new HopperPositionCommand(hopper, Constants.Hopper_SubSystem_Constants.Hopper_Out_Position)),
            AutoFactory.followPath("Left Sweep Steal Path"),
            new DriveOverBumpCommand(drivetrain, heading, driveRequest, directionComing),
            AutoFactory.followPath("Left Shoot Path"),
            //wait for 5 seconds when we should
            new ParallelDeadlineGroup(new WaitCommand(3.375)
                , new FullShootAUTOSequence(drivetrain, shooter, hood, hopperFeeder, shooterFeeder, hopper, 
                    () -> TargetSupplier.getTargetBasedOnAlliance(drivetrain), driveRequest, 
                    ()->true, 
                    () -> SOTMCalc.getInstance().getParameters(
                      drivetrain.getPose().getTranslation()
                          .getDistance(TargetSupplier.getTargetBasedOnAlliance(drivetrain))).rpm(),
                          //hood angle supplier
                           () -> SOTMCalc.getInstance().getParameters(
                      drivetrain.getPose().getTranslation()
                          .getDistance(TargetSupplier.getTargetBasedOnAlliance(drivetrain))).hoodAngle(), intake)),
            //drive back over for second cycle.
            //resets the hopper while driving back over bump
            new ParallelDeadlineGroup(new DriveOverBumpCommand(drivetrain, heading, driveRequest, directionGoing), 
            new HopperPositionCommand(hopper, Constants.Hopper_SubSystem_Constants.Hopper_Out_Position),
            new InstantCommand(()->{shooter.setSpeed(0);}),
            new InstantCommand(()->{shooterFeeder.setSpeed(0);}),
            new InstantCommand(()->{hopperFeeder.setRPMtopFeeder(0);}),
            new InstantCommand(()->{hopperFeeder.setRPMHopper(0);})),
            AutoFactory.followPath("Left Sweep Steal 2 Path"),
            new DriveOverBumpCommand(drivetrain, heading, driveRequest, directionComing),
            AutoFactory.followPath("Left Shoot Path"),
            //Same wait for 5 seconds to shoot
            new ParallelDeadlineGroup( new WaitCommand(4)
                , new FullShootAUTOSequence(drivetrain, shooter, hood, hopperFeeder, shooterFeeder, hopper, 
                    () -> TargetSupplier.getTargetBasedOnAlliance(drivetrain), driveRequest, 
                    ()->true, 
                    () -> SOTMCalc.getInstance().getParameters(
                      drivetrain.getPose().getTranslation()
                          .getDistance(TargetSupplier.getTargetBasedOnAlliance(drivetrain))).rpm(),
                          //hood angle supplier
                           () -> SOTMCalc.getInstance().getParameters(
                      drivetrain.getPose().getTranslation()
                          .getDistance(TargetSupplier.getTargetBasedOnAlliance(drivetrain))).hoodAngle(), intake)),
            //reset hopper after shooting
            new ParallelCommandGroup(new HopperPositionCommand(hopper, Constants.Hopper_SubSystem_Constants.Hopper_Out_Position),
            new InstantCommand(()->{shooter.setSpeed(0);}),
            new InstantCommand(()->{shooterFeeder.setSpeed(0);}),
            new InstantCommand(()->{hopperFeeder.setRPMtopFeeder(0);}),
            new InstantCommand(()->{hopperFeeder.setRPMHopper(0);}))

            
        );
         }, Set.of(drivetrain));

    }
}
