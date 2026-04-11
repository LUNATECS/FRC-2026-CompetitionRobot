// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.theTRENCH;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.autos.AutoFactory;
import frc.robot.autos.DriveOverBumpCommand;
import frc.robot.commands.HopperPositionCommand;
import frc.robot.commands.ShootOnTheMove.SOTM_AUTOSequence;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubSystem;
import frc.robot.subsystems.HopperFeederSubSystem;
import frc.robot.subsystems.HopperSubSystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterFeederSubSystem;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.util.AllianceFliputil;
import frc.robot.util.TargetSupplier;


/** Add your docs here. */
public class RightTrenchSOTMAuto {
    //this is our limey auto
    double maxSpeed = 3.0;        //match AutoFactory constraints
    double maxAngularRate = 2.0;

    public static Command build(CommandSwerveDrivetrain drivetrain, 
        SwerveRequest.FieldCentric driveRequest, 
        HoodSubSystem hood, 
        ShooterSubSystem shooter, 
        HopperFeederSubSystem hopperFeeder, 
        ShooterFeederSubSystem shooterFeeder,
        HopperSubSystem hopper,
        IntakeSubSystem intake,
        LEDSubsystem led) 
        {
        return Commands.defer(()->{
        double directionGoing = AllianceFliputil.getFlipBasedOnAllianceForGoingToCenter();
        double directionComing = AllianceFliputil.getFlipbasedOnAllianceForComingBack();
        Rotation2d headingGoing = AllianceFliputil.getHeadingBasedOnAllianceForBumpRIGHT();
        Rotation2d headingComing = AllianceFliputil.getHeadingBasedOnAllianceForBumpRIGHTHeadingBack();
        Rotation2d headingGoingFLAT = AllianceFliputil.getHeadingForGoingFLAT();

        
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

            new ParallelDeadlineGroup(AutoFactory.followPath("Trench Right P1"), 
            new HopperPositionCommand(hopper, Constants.Hopper_SubSystem_Constants.Hopper_Out_Position)),
            new DriveOverBumpCommand(drivetrain, headingComing, driveRequest, directionComing),
            new ParallelDeadlineGroup(
                AutoFactory.followPath("Trench Right SOTM Path"),
                new SOTM_AUTOSequence(drivetrain, shooter, hood, hopperFeeder, shooterFeeder, hopper, intake, led, ()->TargetSupplier.getTargetBasedOnAlliance(drivetrain), ()->true)
            ),
            AutoFactory.followPath("Trench Right SOTM P2"),
            new ParallelDeadlineGroup(
            new DriveOverBumpCommand(drivetrain, headingComing, driveRequest, directionComing)
            , new InstantCommand(()->{hopperFeeder.setSpeed(-0.2);})),
            new ParallelDeadlineGroup(
                AutoFactory.followPath("Trench Right SOTM Path"),
                new SOTM_AUTOSequence(drivetrain, shooter, hood, hopperFeeder, shooterFeeder, hopper, intake, led, ()->TargetSupplier.getTargetBasedOnAlliance(drivetrain), ()->true)
            ),
            AutoFactory.followPath("Trench Right SOTM P2"),
            new ParallelDeadlineGroup(
            new DriveOverBumpCommand(drivetrain, headingComing, driveRequest, directionComing)
            , new InstantCommand(()->{hopperFeeder.setSpeed(-0.2);})),
                    //()->7.0, intake)),


            //reset hopper after shooting
            new ParallelCommandGroup(new HopperPositionCommand(hopper, Constants.Hopper_SubSystem_Constants.Hopper_Out_Position),
            new InstantCommand(()->{shooter.setSpeed(0);}),
            new InstantCommand(()->{shooterFeeder.setSpeed(0);}),
            new InstantCommand(()->{hopperFeeder.setRPMHopper(0);}))

            
        );
         }, Set.of(drivetrain));

    }
}
