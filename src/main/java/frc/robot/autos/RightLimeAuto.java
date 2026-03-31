// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import static edu.wpi.first.units.Units.Rotation;

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
public class RightLimeAuto {
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
        Rotation2d headingGoing = AllianceFliputil.getHeadingBasedOnAllianceForBumpRIGHT();
        Rotation2d headingComing = AllianceFliputil.getHeadingBasedOnAllianceForBumpRIGHTHeadingBack();
        Rotation2d headingGoingFLAT = AllianceFliputil.getHeadingForGoingFLAT();
        
            return Commands.sequence(
            //     Commands.runOnce(() -> {
            //     try {
            //         var path1 = PathPlannerPath.fromPathFile("Starting Path Right");
            //         //have the make a starting path for these authos
            //         drivetrain.resetPose(path1.getStartingHolonomicPose().get());
            //     } catch (Exception e) {
            //         e.printStackTrace();
            //     }
            // }),

             new ParallelDeadlineGroup(new DriveOverBumpCommand(drivetrain, headingGoing, driveRequest, directionGoing), 
            new HopperPositionCommand(hopper, Constants.Hopper_SubSystem_Constants.Hopper_Out_Position)),
            AutoFactory.followPath("Right V2 Sweep 1"),
            new DriveOverBumpCommand(drivetrain, headingComing, driveRequest, directionComing),
            AutoFactory.followPath("Right Shoot Path V2"),
            //wait for 5 seconds when we should
            new ParallelDeadlineGroup(new WaitCommand(2.875)
                , new FullShootAUTOSequence(drivetrain, shooter, hood, hopperFeeder, shooterFeeder, hopper, 
                    () -> TargetSupplier.getTargetBasedOnAlliance(drivetrain), driveRequest, 
                    ()->true, 
                    //()-> 2600.0,
                     () -> SOTMCalc.getInstance().getParameters(
                       drivetrain.getPose().getTranslation()
                           .getDistance(TargetSupplier.getTargetBasedOnAlliance(drivetrain))).rpm()-100,
                          //hood angle supplier
                            () -> SOTMCalc.getInstance().getParameters(
                       drivetrain.getPose().getTranslation()
                           .getDistance(TargetSupplier.getTargetBasedOnAlliance(drivetrain))).hoodAngle() - 2, intake)),
                    //()->7.0, intake)), 
                    //heeyyyyy adiiiii slayyyyyyyyyyy yasssssss

            //drive back over for second cycle.
           new ParallelDeadlineGroup(new DriveOverBumpCommand(drivetrain, headingGoingFLAT, driveRequest, directionGoing), 
            new HopperPositionCommand(hopper, Constants.Hopper_SubSystem_Constants.Hopper_Out_Position),
            new InstantCommand(()->{shooter.setSpeed(0);}),
            new InstantCommand(()->{shooterFeeder.setSpeed(0);}),
            new InstantCommand(()->{hopperFeeder.setRPMHopper(0);})),
            AutoFactory.followPath("Right V2 Sweep 2"),
            new ParallelDeadlineGroup(
            new DriveOverBumpCommand(drivetrain, headingComing, driveRequest, directionComing)
            , new InstantCommand(()->{hopperFeeder.setSpeed(-0.2);})),
            AutoFactory.followPath("Right Shoot Path V2"),
            //Same wait for 5 seconds to shoot
            new ParallelDeadlineGroup(new WaitCommand(3.0)
                , new FullShootAUTOSequence(drivetrain, shooter, hood, hopperFeeder, shooterFeeder, hopper, 
                    () -> TargetSupplier.getTargetBasedOnAlliance(drivetrain), driveRequest, 
                    ()->true, 
                    //()-> 2600.0,

                     () -> SOTMCalc.getInstance().getParameters(
                       drivetrain.getPose().getTranslation()
                           .getDistance(TargetSupplier.getTargetBasedOnAlliance(drivetrain))).rpm() - 100,
                          //hood angle supplier
                            () -> SOTMCalc.getInstance().getParameters(
                       drivetrain.getPose().getTranslation()
                           .getDistance(TargetSupplier.getTargetBasedOnAlliance(drivetrain))).hoodAngle() - 2, intake)),
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
