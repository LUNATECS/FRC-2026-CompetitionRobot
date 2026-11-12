// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autos.AutoFactory;
import frc.robot.commands.HoodTrenchCommand;
import frc.robot.commands.HopperPositionCommand;
import frc.robot.commands.ShooterDynamicVelocityCommand;
import frc.robot.util.AllianceFliputil;
import frc.robot.util.HubTracker;
import frc.robot.util.TrenchPositionSupplier;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private double degreesOutOfAuto;
  private boolean alreadySetForStartOfAuto;
  private Command builtAuto = null;
  private Command lastSelected = null;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    this.degreesOutOfAuto = 0;
    this.alreadySetForStartOfAuto = false;
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */

  @Override
  public void driverStationConnected() {
    m_robotContainer.setupAutoChooser();
    Alliance alliance = DriverStation.getAlliance().get();
    Rotation2d startRotation = m_robotContainer.getAutoStartRotation(alliance);
    m_robotContainer.getSwerveDrivetrain().resetPose(new Pose2d(
        m_robotContainer.getSwerveDrivetrain().getPose().getTranslation(),
        startRotation
    ));
    m_robotContainer.getSwerveDrivetrain().resetGyro(startRotation.getDegrees());
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Match Timer", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Match Timer (0-160)", HubTracker.getMatchTime());
    SmartDashboard.putBoolean("Hub Active", HubTracker.isActive());
    SmartDashboard.putString("Current Shift", HubTracker.getCurrentShift().toString());
    SmartDashboard.putNumber("Time left in Current Shift", HubTracker.timeRemainingInCurrentShift().orElse(Seconds.zero()).in(Seconds));
    SmartDashboard.putString("Alliance", DriverStation.getAlliance().get().toString());

    SmartDashboard.putNumber("Alliance drive value", RobotContainer.AllianceDriveValue);

    SmartDashboard.putNumber("Alliance Flip util Forward Backward", AllianceFliputil.getFlipBasedOnAllianceForGoingToCenter());
    SmartDashboard.putNumber("Alliance Flip util Going rotation", AllianceFliputil.getHeadingBasedOnAllianceForBumpRIGHT().getDegrees());

  

    SmartDashboard.putBoolean("Under Trench?", TrenchPositionSupplier.getTrenchZone(m_robotContainer.getSwerveDrivetrain()));

    SmartDashboard.putNumber("Auto Starting Rotation", m_robotContainer.getAutoStartRotation(DriverStation.getAlliance().get()).getDegrees());


    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void robotInit() {
    AutoFactory.preloadPaths();
    //Pathfinding.setPathfinder(new CustomPathfinder());
    //ONLY ENABLE THE CUSTOM PATHFINDER WHEN WE ARE READY TO, DON'T DO IT BEFORE DCMP
    //MAKE SURE TO CHANGE THIS ON STARTUP IF WE CHANGE THE AUTO STARTING ORIENTATION
    //we need the warmup command, so there is is, using the non-depracated version
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
  }

  @Override
  public void disabledInit() {
    m_robotContainer.updateSelectedSide(); 

    if (!alreadySetForStartOfAuto && DriverStation.getAlliance().isPresent()){
    Alliance alliance = DriverStation.getAlliance().get();
    Rotation2d startRotation = m_robotContainer.getAutoStartRotation(alliance);
    m_robotContainer.getSwerveDrivetrain().resetPose(new Pose2d(
        m_robotContainer.getSwerveDrivetrain().getPose().getTranslation(),
        startRotation
    ));
    m_robotContainer.getSwerveDrivetrain().resetGyro(startRotation.getDegrees());
    alreadySetForStartOfAuto = true;
    }
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.updateSelectedSide(); 
    boolean allowUpdate = SmartDashboard.getBoolean("ALLOW AUTO POSE UPDATE", true);

    if (allowUpdate && DriverStation.getAlliance().isPresent()) {
        var alliance = DriverStation.getAlliance().get();
        Rotation2d startRotation = m_robotContainer.getAutoStartRotation(alliance);

        m_robotContainer.getSwerveDrivetrain().resetPose(
            new Pose2d(
                m_robotContainer.getSwerveDrivetrain().getPose().getTranslation(),
                startRotation
            )
        );

        m_robotContainer.getSwerveDrivetrain().resetGyro(startRotation.getDegrees());

        Command selected = m_robotContainer.getAutoChooser().getSelected();

        if (selected != lastSelected) {
            lastSelected = selected;
            builtAuto = selected; // if already concrete
        }
        //adding lazy load of paths
        //Command selected = m_robotContainer.getAutoChooser().getSelected();
        //if (selected != m_autonomousCommand){
        //  m_autonomousCommand = selected;
        //  if(m_robotContainer.getSelectedSide() == AutoSide.LEFT){
        //  }

       // }
        


      }
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
      RobotContainer.AllianceDriveValue = 1;
    }
    else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){
      RobotContainer.AllianceDriveValue = 1;
    }
    //SmartDashboard.putNumber("Alliance drive value", RobotContainer.AllianceDriveValue);

    //if (DriverStation.getAlliance().get() == Alliance.Red){
      //m_robotContainer.getSwerveDrivetrain().resetPose(new Pose2d(m_robotContainer.getSwerveDrivetrain().getPose().getX(), m_robotContainer.getSwerveDrivetrain().getPose().getY(), Rotation2d.k180deg));
    //}
    


  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //SmartDashboard.putBoolean("ALLOW AUTO POSE UPDATE", false);
    //m_robotContainer.updateSelectedSide(); 
    
    //CSA commented out this line
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    //    var alliance = DriverStation.getAlliance().get();

    //Rotation2d autoRotation = m_robotContainer.getAutoStartRotation(alliance);

    //Pose2d startPose = new Pose2d(
    //    m_robotContainer.getSwerveDrivetrain().getPose().getTranslation(),
    //    autoRotation
    //);

    //m_robotContainer.getSwerveDrivetrain().resetPose(startPose);
    //m_robotContainer.getSwerveDrivetrain().resetGyro(autoRotation.getDegrees());
    
    /*var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        m_robotContainer.getSwerveDrivetrain().resetGyro(180);
    } else {
        m_robotContainer.getSwerveDrivetrain().resetGyro(0);
    }
        */

    //CommandScheduler.getInstance().schedule(new HoodHoldCommand(new HoodSubSystem(), ()->1.5));

    // schedule the autonomous command (example)
    m_autonomousCommand = builtAuto;
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit(){ 
    degreesOutOfAuto = m_robotContainer.getSwerveDrivetrain().getPose().getRotation().getDegrees();   
    CommandScheduler.getInstance().schedule(new HopperPositionCommand(m_robotContainer.getHopper(), Constants.Hopper_SubSystem_Constants.Hopper_Out_Position));
    CommandScheduler.getInstance().schedule(new InstantCommand(()->{m_robotContainer.getHopperFeeder().setSpeed(0);}));
    CommandScheduler.getInstance().schedule(new InstantCommand(()->{m_robotContainer.getShooterFeeder().setSpeed(0);}));
  }

  @Override
  public void teleopInit() {

    //CHANGE SPEED OF IDLE SHOOTER HERE
    m_robotContainer.getShooter().setDefaultCommand(new ShooterDynamicVelocityCommand(m_robotContainer.getShooter(), ()->1000.0));
    m_robotContainer.getHood().setDefaultCommand(new HoodTrenchCommand(m_robotContainer.getHood(), m_robotContainer.getSwerveDrivetrain()));
    //m_robotContainer.getSwerveDrivetrain().resetRotation(Rotation2d.fromDegrees(degreesOutOfAuto));
    //m_robotContainer.getSwerveDrivetrain().resetGyro(degreesOutOfAuto);
    //Pose2d endAutoPose = new Pose2d(
    //    m_robotContainer.getSwerveDrivetrain().getPose().getTranslation(),
    //    Rotation2d.fromDegrees(degreesOutOfAuto)
    //);
    
    //m_robotContainer.getSwerveDrivetrain().resetPose(endAutoPose);
    //m_robotContainer.getSwerveDrivetrain().resetGyro(degreesOutOfAuto);



    //puts the hopper out at the beginning of teleop.
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /* 
    SmartDashboard.putString("Teleop Time", ""+HubTracker.timeRemainingInCurrentShift().orElse(Seconds.zero()).in(Seconds));
    if(HubTracker.timeRemainingInCurrentShift().orElse(Seconds.zero()).in(Seconds) < 10.0 && HubTracker.timeRemainingInCurrentShift().orElse(Seconds.zero()).in(Seconds) > 9.0){
      this.m_robotContainer.getController().getHID().setRumble(RumbleType.kBothRumble, 0.5);
    } else if(HubTracker.timeRemainingInCurrentShift().orElse(Seconds.zero()).in(Seconds) < 9.0 && HubTracker.timeRemainingInCurrentShift().orElse(Seconds.zero()).in(Seconds) > 8.0){
      this.m_robotContainer.getController().getHID().setRumble(RumbleType.kBothRumble, 0);
    } else if(HubTracker.timeRemainingInCurrentShift().orElse(Seconds.zero()).in(Seconds) < 5.0 && HubTracker.timeRemainingInCurrentShift().orElse(Seconds.zero()).in(Seconds) > 4.0){
      this.m_robotContainer.getController().getHID().setRumble(RumbleType.kBothRumble, 1);
    } else if(HubTracker.timeRemainingInCurrentShift().orElse(Seconds.zero()).in(Seconds) < 4.0 && HubTracker.timeRemainingInCurrentShift().orElse(Seconds.zero()).in(Seconds) > 3.0){
      this.m_robotContainer.getController().getHID().setRumble(RumbleType.kBothRumble, 0);
    } 
      */
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
