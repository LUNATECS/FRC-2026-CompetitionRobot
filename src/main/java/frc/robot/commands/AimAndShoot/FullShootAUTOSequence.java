// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AimAndShoot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.HoodDynamicPointCommand;
import frc.robot.commands.HopperAgitateTrigerredCommand;
import frc.robot.commands.ShooterDynamicVelocityCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubSystem;
import frc.robot.subsystems.HopperFeederSubSystem;
import frc.robot.subsystems.HopperSubSystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.ShooterFeederSubSystem;
import frc.robot.subsystems.ShooterSubSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullShootAUTOSequence extends ParallelCommandGroup {
  /** Creates a new FullShootAUTOSequence. */
  public FullShootAUTOSequence(
    CommandSwerveDrivetrain drivetrain,
        ShooterSubSystem shooter,
        HoodSubSystem hood,
        HopperFeederSubSystem hopperFeeder,
        ShooterFeederSubSystem shooterFeeder,
        HopperSubSystem hopper,
        Supplier<Translation2d> targetPos,
        SwerveRequest.FieldCentric driveRequest,
        BooleanSupplier fireButton,
        Supplier<Double> shooterRPMSupplier,
        Supplier<Double> hoodAngleSupplier,
        IntakeSubSystem intake
  ) {
        FaceFieldPointCommand drivetrainCommand = new FaceFieldPointCommand(drivetrain, () -> new Translation2d(0, 0), targetPos, driveRequest);
        HoodDynamicPointCommand hoodCommand = new HoodDynamicPointCommand(hood, hoodAngleSupplier);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShooterDynamicVelocityCommand(shooter, shooterRPMSupplier),
      hoodCommand,
      drivetrainCommand,
      new ShootCommand(hopperFeeder, shooterFeeder, shooter, hoodCommand, intake, fireButton),
      new HopperAgitateTrigerredCommand(hopper, fireButton, shooter, hoodCommand)
    ); 
  }
}
