// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.PracticeAuto;
//import frc.robot.commands.IntakePivotCommand;
import frc.robot.commands.ShooterBangBangControlCommand;
import frc.robot.commands.ShooterVelocityCommand;
import frc.robot.subsystems.ClimberSubSystem;
import frc.robot.subsystems.HoodSubSystem;
import frc.robot.subsystems.HopperFeederSubSystem;
import frc.robot.subsystems.HopperSubSystem;
//import frc.robot.subsystems.IntakePivotSubSystem;
//import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.ShooterFeederSubSystem;
import frc.robot.subsystems.ShooterSubSystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ShooterSubSystem shooter = new ShooterSubSystem();
  private final ShooterFeederSubSystem shooterFeeder = new ShooterFeederSubSystem();
  //private final IntakeSubSystem intake = new IntakeSubSystem();
  //private final IntakePivotSubSystem intakePivot = new IntakePivotSubSystem();
  private final HopperSubSystem hopper = new HopperSubSystem();
  private final HopperFeederSubSystem hopperFeeder = new HopperFeederSubSystem();
  private final HoodSubSystem hood = new HoodSubSystem();
  private final ClimberSubSystem climber = new ClimberSubSystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS5Controller driver = new CommandPS5Controller(Constants.DriverConstants.DRIVER_CONTROLLER_PORT);
  private final CommandPS5Controller tester = new CommandPS5Controller(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first
   * .wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //  .onTrue(new ExampleCommand(m_exampleSubsystem));

    //TESTING SUBSYSTEMS WITH TESTER CONTROLLER
    tester.square().onTrue(new InstantCommand(() -> {
    hopperFeeder.setSpeed(1.5);
    //intake.setSpeed(1);
    shooterFeeder.setSpeed(-1);
  }))
    .onFalse(new InstantCommand(()->{
      hopperFeeder.setSpeed(0);
      //intake.setSpeed(0);
      shooterFeeder.setSpeed(0);
  }));

  tester.triangle().onTrue(new InstantCommand(() -> {
    hopperFeeder.setSpeed(-1.5);
    //intake.setSpeed(-1);
    shooterFeeder.setSpeed(1);
  }))
    .onFalse(new InstantCommand(()->{
      hopperFeeder.setSpeed(0);
      //intake.setSpeed(0);
      shooterFeeder.setSpeed(0);
  }));

  //shooter testing
  tester.povUp().onTrue(new InstantCommand(()->{shooter.setSpeed(0.25);}, shooter))
  .onFalse(new InstantCommand(()->{shooter.setSpeed(0);}, shooter));

  tester.povLeft().onTrue(new InstantCommand(()->{shooter.setSpeed(1);}))
  .onFalse(new InstantCommand(()->{shooter.setSpeed(0);}));

  tester.povDown().onTrue(new InstantCommand(()->{shooter.setSpeed(-0.5);}, shooter))
  .onFalse(new InstantCommand(()->{shooter.setSpeed(0);}, shooter));
  tester.povRight().onTrue(new InstantCommand(()->{shooter.setSpeed(-1);}, shooter))
  .onFalse(new InstantCommand(()->{shooter.setSpeed(0);}, shooter));


  //Hood Testing
  tester.R1().onTrue(new InstantCommand(()->{hood.setSpeed(0.15);}))
    .onFalse(new InstantCommand(()->{hood.setSpeed(0);}));

  tester.L1().onTrue(new InstantCommand(()->{hood.setSpeed(-0.15);}))
    .onFalse(new InstantCommand(()->{hood.setSpeed(0);}));



  tester.R2().onTrue(new InstantCommand(()->{hopper.setSpeed(0.5);}))
    .onFalse(new InstantCommand(()->{hopper.setSpeed(0);}));

  tester.L2().onTrue(new InstantCommand(()->{hopper.setSpeed(-0.5);}))
    .onFalse(new InstantCommand(()->{hopper.setSpeed(0);}));  

  //shooter commands testing
  tester.cross().onTrue(new ShooterVelocityCommand(shooter, 1000));
  tester.circle().onTrue(new ShooterBangBangControlCommand(shooter, 1000));

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return PracticeAuto.build();
  }
}
