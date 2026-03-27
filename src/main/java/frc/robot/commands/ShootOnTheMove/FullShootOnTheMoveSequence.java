// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootOnTheMove;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.HoodDynamicPointCommand;
import frc.robot.commands.ShooterDynamicVelocityCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubSystem;
import frc.robot.subsystems.HopperFeederSubSystem;
import frc.robot.subsystems.HopperSubSystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterFeederSubSystem;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.util.SOTMSuppliers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullShootOnTheMoveSequence extends ParallelCommandGroup {
  
  /** Creates a new FullShootOnTheMoveSequence. */
  public FullShootOnTheMoveSequence(
        CommandSwerveDrivetrain drivetrain,
        ShooterSubSystem shooter,
        HoodSubSystem hood,
        HopperFeederSubSystem hopperFeeder,
        ShooterFeederSubSystem shooterFeeder,
        HopperSubSystem hopper,
        IntakeSubSystem intake,
        LEDSubsystem led,
        Supplier<Translation2d> robotTranslation,
        Supplier<Translation2d> targetPos,
        SwerveRequest.FieldCentric driveRequest,
        BooleanSupplier fireButton
  ) {
    SOTMSuppliers suppliers = new SOTMSuppliers(drivetrain, shooter, hood, targetPos);
      
    

      //used commands
      HoodDynamicPointCommand hoodCommand = new HoodDynamicPointCommand(hood, suppliers.hoodAngleSupplier);

        ShooterDynamicVelocityCommand shooterCommand = new ShooterDynamicVelocityCommand(shooter, suppliers.shooterRPMSupplier);
        ShootOnTheMoveCommand drivetrainCommand = new ShootOnTheMoveCommand(
                drivetrain, robotTranslation, targetPos, suppliers.flightTimeSupplier, driveRequest
        );
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      drivetrainCommand,
      shooterCommand,
      hoodCommand,
      new ShootOnTheMoveShootCommand(hopperFeeder, shooterFeeder, shooter, hoodCommand, drivetrainCommand, intake, led, fireButton)
      //new HopperAgitateTrigerredCommand(hopper, fireButton, shooter, hoodCommand)
    );
  }
}
