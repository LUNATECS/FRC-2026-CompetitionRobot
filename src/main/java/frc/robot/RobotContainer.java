// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.DriveOverBumpCommand;
import frc.robot.autos.LeftLimeAuto;
import frc.robot.autos.LeftSweepAuto;
import frc.robot.autos.LeftTwoCycleAuto;
import frc.robot.autos.LeftTwoCycleDynamicAuto;
import frc.robot.autos.LeftTwoCycleSweepAuto;
import frc.robot.autos.LeftTwoSweepDynamicAuto;
import frc.robot.autos.PracticeAuto;
import frc.robot.autos.RightLimeAuto;
import frc.robot.autos.RightSweepAuto;
import frc.robot.autos.RightTwoCycleAuto;
import frc.robot.autos.RightTwoCycleDynamicAuto;
import frc.robot.autos.RightTwoCycleSweepAuto;
import frc.robot.autos.RightTwoSweepDynamicAuto;
import frc.robot.commands.AimAndShoot.FaceFieldPointCommand;
import frc.robot.commands.HoodHoldCommand;
import frc.robot.commands.HopperAgitateCommand;
import frc.robot.commands.HopperPositionCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootOnTheMove.FullShootOnTheMoveSequence;
import frc.robot.commands.ShooterVelocityCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubSystem;
import frc.robot.subsystems.HopperFeederSubSystem;
import frc.robot.subsystems.HopperSubSystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.ShooterFeederSubSystem;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.util.TargetSupplier;

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
  private final IntakeSubSystem intake = new IntakeSubSystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS5Controller driver = new CommandPS5Controller(Constants.DriverConstants.DRIVER_CONTROLLER_PORT);
  private final CommandPS5Controller operator = new CommandPS5Controller(1);
  private final CommandPS5Controller tester = new CommandPS5Controller(2);
  //private final CommandPS5Controller tester = new CommandPS5Controller(2);

  private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //autochooser
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    //private final SendableChooser<Command> autoChooser;

    //shooting configs/targets
    //private Pose2d targetPose = new Pose2d(4.625, 4.04, new Rotation2d(0)); // Example target pose on the field

    public static Field2d virtualTargetField = new Field2d();

    public static double AllianceDriveValue = 0;



  public void setupAutoChooser() {
        autoChooser.setDefaultOption("DO NOTHING", PracticeAuto.build(drivetrain));
        //autoChooser.addOption("Right One Cycle Climb", RightOneCycleClimbAuto.build(drivetrain, drive, hopper));
        //two cycle autos
        autoChooser.addOption("Right Two Cycle Auto", RightTwoCycleAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));
        autoChooser.addOption("Left Two Cycle Auto", LeftTwoCycleAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));
        //one cycle autos
        //autoChooser.addOption("Right One Cycle Auto", RightOneCycleAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));
        //autoChooser.addOption("Left One Cycle Auto", LeftOneCycleAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));
        //sweep autos
        autoChooser.addOption("Left Sweep Auto", LeftSweepAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));
        autoChooser.addOption("Right Sweep Auto", RightSweepAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));

        //double sweep auto
        autoChooser.addOption("Double Left Sweep Auto", LeftTwoCycleSweepAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));
        autoChooser.addOption("Double Right Sweep Auto", RightTwoCycleSweepAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));

        //depot autos
        //autoChooser.addOption("Left Depot Auto", LeftDepotAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));
        //autoChooser.addOption("Right Depot Auto", RightDepotAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));


        //alliance tune auto
        //autoChooser.addOption("Alliance Tuning Auto", AllianceTuningAuto.build(drivetrain, drive));
        //autoChooser.addOption("Driver over bump test", DrivingOverBumpTestAuto.build(drivetrain, drive));

        //steal autos
        //autoChooser.addOption("Right Two Cycle STEAL Auto", RightTwoCycleSTEALAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));
        //autoChooser.addOption("Left Two Cycle STEAL Auto", LeftTwoCycleSTEALAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));


        //grief autos
        //autoChooser.addOption("Left Two Cycle Grief Auto", LeftTwoCycleGriefAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));
        //autoChooser.addOption("Right Two Cycle Grief Auto", RightTwoCycleGriefAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));


        //practice object detection 
        //autoChooser.addOption("Fuel Loop Auto Obj.Detec", DriveToFuelLoopAuto.build(drivetrain, intake));
        //Dynamic Sweep Autos
        autoChooser.addOption("Right Two Sweep Dynamic Auto", RightTwoSweepDynamicAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));
        autoChooser.addOption("Left Two Sweep Dynamic Auto", LeftTwoSweepDynamicAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));

        //Dynamic two cycle autos
        autoChooser.addOption("Right Two Cycle Dynamic Auto", RightTwoCycleDynamicAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));
        autoChooser.addOption("Left Two Cycle Dynamic Auto", LeftTwoCycleDynamicAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));

        //limey citrus circuits type autos
        autoChooser.addOption("Left Limey Auto", LeftLimeAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));
        autoChooser.addOption("Right Limey Auto", RightLimeAuto.build(drivetrain, drive, hood, shooter, hopperFeeder, shooterFeeder, hopper, intake));

        // Put the chooser on SmartDashboard
        SmartDashboard.putData("Autonomous Mode FINAL", autoChooser);
    }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putNumber("shooterRPMInput", 0);
    SmartDashboard.putNumber("hoodInputAngle", 0);
    SmartDashboard.putData(virtualTargetField);

    NamedCommands.registerCommand("Drive Over Bump RIGHT", new DriveOverBumpCommand(drivetrain, Rotation2d.fromDegrees(-45), drive, 1.0));
    NamedCommands.registerCommand("Drive Over Bump LEFT", new DriveOverBumpCommand(drivetrain, Rotation2d.fromDegrees(-45), drive, -1.0));

    NamedCommands.registerCommand("Start Intake Command", new IntakeCommand(intake, 6500));

    NamedCommands.registerCommand("StopIntakeCommand", new InstantCommand(()->{intake.setSpeed(0);}, intake));

    // Configure the trigger bindings
    //autoChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData("Auto Mode FInal", autoChooser);
    configureBindings();
    //setupAutoChooser();

  }

  public CommandPS5Controller getController(){
    return driver;
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
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed * AllianceDriveValue) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed * AllianceDriveValue) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate * 1.5) // Drive counterclockwise with negative X (left)
            )
        );

      shooter.setDefaultCommand(new ShooterVelocityCommand(shooter, ()->3000.0));
      //hood.setDefaultCommand(new HoodHoldCommand(hood, ()->1.5));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //driver.L2().whileTrue(drivetrain.applyRequest(() -> brake));
        //driver.L1().whileTrue(drivetrain.applyRequest(() ->
        //    point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        //));

        //driver.povUp().onTrue(new DriveOverBumpCommand(drivetrain, Rotation2d.fromDegrees(-45), drive, 1));
                //driver.povDown().onTrue(new DriveOverBumpCommand(drivetrain, Rotation2d.fromDegrees(-45), drive, -1));

        driver.L1().whileTrue(
          new FullShootOnTheMoveSequence(
            drivetrain, 
            shooter, 
            hood, 
            hopperFeeder, 
            shooterFeeder,
            hopper, 
            intake,
            () -> new Translation2d(
                    -driver.getLeftY() * MaxSpeed,
                    -driver.getLeftX() * MaxSpeed
                ), 
            () -> TargetSupplier.getTargetBasedOnAlliance(drivetrain), 
            drive,
            () -> driver.R2().getAsBoolean() 
            )
        );//
        //.onFalse(new HopperPositionCommand(hopper, Constants.Hopper_SubSystem_Constants.Hopper_Out_Position));
        //after command is over, then reset hoppery back to position)

        driver.L2().onTrue(new InstantCommand(()->{hopperFeeder.setSpeed(-0.5);}))
          .onFalse(new InstantCommand(()->{hopperFeeder.setSpeed(0);}));


        driver.L2().onTrue(new InstantCommand(()->{shooterFeeder.setSpeed(-0.5);}))
          .onFalse(new InstantCommand(()->{shooterFeeder.setSpeed(0);}));

        /*driver.R1().whileTrue(
            new FullShootSequence(
                drivetrain,
                shooter,
                hood,
                hopperFeeder,
                shooterFeeder,
                // Robot position
                () -> new Translation2d(
                    -driver.getLeftY() * MaxSpeed,
                    -driver.getLeftX() * MaxSpeed
                ),
                // Target position
                () -> TargetSupplier.getTargetBasedOnAlliance(drivetrain),
                drive,
                // Fire button
                () -> driver.R2().getAsBoolean(),
                // Shooter RPM supplier
                () -> SOTMCalc.getInstance().getParameters(
                      drivetrain.getPose().getTranslation()
                          .getDistance(TargetSupplier.getTargetBasedOnAlliance(drivetrain))).rpm(),
                // Hood angle supplier
                () -> SOTMCalc.getInstance().getParameters(
                      drivetrain.getPose().getTranslation()
                          .getDistance(TargetSupplier.getTargetBasedOnAlliance(drivetrain))).hoodAngle()
                          
                //() -> ShotCalc.getShot(
                        //drivetrain.getPose().getTranslation()
                                //.getDistance(targetPose.getTranslation())).rpm,

                //()  -> ShotCalc.getShot(
                        //drivetrain.getPose().getTranslation()
                                //.getDistance(targetPose.getTranslation())).hoodDeg
            )
        );
        */

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //driver.triangle().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //driver.cross().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //driver.povUp().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //driver.povDown().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        //driver.circle().onTrue(drivetrain.runOnce(() -> drivetrain.resetRotation(Rotation2d.fromDegrees(0))));

        driver.R1().onTrue(new IntakeCommand(intake, 6500))
          .onFalse(new InstantCommand(()->{intake.setSpeed(0);}, intake) );
        driver.triangle().onTrue(new HopperPositionCommand(hopper, Constants.Hopper_SubSystem_Constants.Hopper_Out_Position));

        driver.cross().onTrue(new HopperAgitateCommand(hopper));


        driver.circle().onTrue(
            drivetrain.runOnce(() -> {
                drivetrain.resetGyro(0);

                Pose2d newPose = new Pose2d(
                    drivetrain.getPose().getTranslation(),
                    Rotation2d.kZero
                );

                drivetrain.resetPose(newPose);  
                drivetrain.getState().Pose.equals(newPose);
            })
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        driver.touchpad().whileTrue(
            new FaceFieldPointCommand(
                drivetrain,
                () -> new Translation2d(
                    -driver.getLeftY() * MaxSpeed,
                    -driver.getLeftX() * MaxSpeed
                ),
                () -> TargetSupplier.getTargetBasedOnAlliance(drivetrain),
                drive
            )
        );


    //operator controls
    //Hood Testing
    operator.R1().onTrue(new HoodHoldCommand(hood, ()->7.5));

    operator.R1().onTrue(new HoodHoldCommand(hood, ()->15));




    operator.R2().whileTrue(new InstantCommand(()->{hopper.setSpeed(0.5);}, hopper))
      .onFalse(new InstantCommand(()->{hopper.setSpeed(0);}, hopper));

    operator.L2().whileTrue(new InstantCommand(()->{hopper.setSpeed(-0.5);}, hopper))
      .onFalse(new InstantCommand(()->{hopper.setSpeed(0);}, hopper));  

    operator.triangle().onTrue(new HopperPositionCommand(hopper, Constants.Hopper_SubSystem_Constants.Hopper_Out_Position));
    operator.cross().onTrue(new HopperAgitateCommand(hopper));

    operator.povLeft().onTrue(new ShooterVelocityCommand(shooter, ()->3000));
    operator.povUp().onTrue(new ShooterVelocityCommand(shooter, ()->3500));
    operator.povRight().onTrue(new ShooterVelocityCommand(shooter, ()->3150));
    operator.povDown().onTrue(new ShooterVelocityCommand(shooter, ()->3300));


    //operator.povDown().onTrue(new InstantCommand(()->{shooter.setRPM(0);}, shooter));



    operator.square().onTrue(new InstantCommand(() -> {
    hopperFeeder.setSpeed(1);
    //intake.setSpeed(1);
    shooterFeeder.setRPMBasedOnShooter(shooter.getRPM());
  }));

    operator.circle().onTrue(
      new InstantCommand(()->{
        hopperFeeder.setSpeed(0);
        shooterFeeder.setSpeed(0);
      })
    );
  //   operator.square().whileTrue(new InstantCommand(() -> {
  //   hopperFeeder.setSpeed(1);
  //   //intake.setSpeed(1);
  //   shooterFeeder.setSpeed(0.6);
  // }, hopperFeeder, shooterFeeder, intake))
  //   .onFalse(new InstantCommand(()->{
  //     hopperFeeder.setSpeed(0);
  //     //intake.setSpeed(0);
  //     shooterFeeder.setSpeed(0);

  // }, hopperFeeder, shooterFeeder, intake));

  // operator.circle().whileTrue(new InstantCommand(() -> {
  //   hopperFeeder.setSpeed(-1);
  //   intake.setSpeed(-1);
  // }, hopperFeeder, shooterFeeder, intake))
  //   .onFalse(new InstantCommand(()->{
  //     hopperFeeder.setSpeed(0);
  //     intake.setSpeed(0);
  // }, hopperFeeder, shooterFeeder, intake));

  operator.touchpad().onTrue(
    new InstantCommand(()->{RobotContainer.AllianceDriveValue = RobotContainer.AllianceDriveValue*-1;})
  );

  

      

    //TESTER CONTROLS:

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //  .onTrue(new ExampleCommand(m_exampleSubsystem));

    //TESTING SUBSYSTEMS WITH TESTER CONTROLLER
     
    tester.square().onTrue(new InstantCommand(() -> {
    hopperFeeder.setSpeed(1);
    //intake.setSpeed(1);
    shooterFeeder.setRPMBasedOnShooter(shooter.getRPM());
  }));

    tester.circle().onTrue(
      new InstantCommand(()->{
        hopperFeeder.setSpeed(0);
        shooterFeeder.setSpeed(0);
      })
    );


    //.onFalse(new InstantCommand(()->{
      //hopperFeeder.setSpeed(0);
      //intake.setSpeed(0);
      //shooterFeeder.setSpeed(0);

  //}));

  tester.triangle().onTrue(new InstantCommand(() -> {
    hopperFeeder.setSpeed(-1.5);
    //intake.setSpeed(-1);
    shooterFeeder.setSpeed(-1);
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
  tester.cross().onTrue(new ShooterVelocityCommand(shooter, ()->SmartDashboard.getNumber("shooterRPMInput", 0)));

  tester.touchpad().onTrue(new HoodHoldCommand(hood,()->SmartDashboard.getNumber("hoodInputAngle", 0)));
  

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public CommandSwerveDrivetrain getSwerveDrivetrain(){
    return drivetrain;
  }
  public HopperSubSystem getHopper(){
    return hopper;
  }
  public ShooterFeederSubSystem getShooterFeeder(){
    return shooterFeeder;
  }
  public HopperFeederSubSystem getHopperFeeder(){
    return hopperFeeder;
  }
}
