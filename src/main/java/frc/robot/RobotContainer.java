// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.autos.PracticeAuto;
import frc.robot.commands.FullShootSequence;
import frc.robot.commands.ShooterBangBangControlCommand;
import frc.robot.commands.ShooterVelocityCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubSystem;
import frc.robot.subsystems.HopperFeederSubSystem;
import frc.robot.subsystems.HopperSubSystem;
import frc.robot.subsystems.ShooterFeederSubSystem;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.util.ShotCalc;

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

  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //shooting configs/targets
    private Pose2d targetPose = new Pose2d(4, 3, new Rotation2d(0)); // Example target pose on the field





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
    //DRIVER CONTROLS, CURRENTLY WITH SYSID TESTING
    drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driver.L2().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.L1().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        driver.R1().whileTrue(
            new FullShootSequence(
                drivetrain,
                shooter,
                hood,
                hopperFeeder,
                shooterFeeder,
                // Robot position
                () -> drivetrain.getState().Pose.getTranslation(),
                // Target position
                () -> targetPose.getTranslation(),
                drive,
                // Fire button
                () -> driver.R2().getAsBoolean(),
                // Shooter RPM supplier
                () -> ShotCalc.getShot(
                        drivetrain.getState().Pose.getTranslation()
                                .getDistance(targetPose.getTranslation())).rpm,
                // Hood angle supplier
                () -> ShotCalc.getShot(
                        drivetrain.getState().Pose.getTranslation()
                                .getDistance(targetPose.getTranslation())).hoodDeg
            )
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.triangle().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.cross().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.povUp().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.povDown().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driver.circle().onTrue(drivetrain.runOnce(() -> drivetrain.resetRotation(Rotation2d.fromDegrees(0))));


        drivetrain.registerTelemetry(logger::telemeterize);


    //TESTER CONTROLS:

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
