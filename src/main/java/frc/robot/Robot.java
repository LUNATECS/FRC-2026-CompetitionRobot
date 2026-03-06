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
import frc.robot.commands.HopperPositionCommand;
import frc.robot.util.HubTracker;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private double degreesOutOfAuto;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    this.degreesOutOfAuto = 0;
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
    double targetRotationDegrees = (alliance == Alliance.Red) ? 180 : 0;
    m_robotContainer.getSwerveDrivetrain().resetRotation(Rotation2d.fromDegrees(targetRotationDegrees));
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Match Timer", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Match Timer (0-160)", HubTracker.getMatchTime());
    SmartDashboard.putBoolean("Hub Active", HubTracker.isActive());
    SmartDashboard.putString("Current Shift", HubTracker.getCurrentShift().toString());
    SmartDashboard.putNumber("Time left in Current Shift", HubTracker.timeRemainingInCurrentShift().orElse(Seconds.zero()).in(Seconds));
    SmartDashboard.putString("Alliance", DriverStation.getAlliance().get().toString());
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void robotInit() {
    //MAKE SURE TO CHANGE THIS ON STARTUP IF WE CHANGE THE AUTO STARTING ORIENTATION
    //we need the warmup command, so there is is, using the non-depracated version
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
  }

  @Override
  public void disabledInit() {
    Alliance alliance = DriverStation.getAlliance().get();
    Rotation2d startRotation = (alliance == Alliance.Red) ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);
    m_robotContainer.getSwerveDrivetrain().resetPose(new Pose2d(
        m_robotContainer.getSwerveDrivetrain().getPose().getTranslation(),
        startRotation
    ));
    m_robotContainer.getSwerveDrivetrain().resetGyro(startRotation.getDegrees());

  }

  @Override
  public void disabledPeriodic() {
    //if (DriverStation.getAlliance().get() == Alliance.Red){
      //m_robotContainer.getSwerveDrivetrain().resetPose(new Pose2d(m_robotContainer.getSwerveDrivetrain().getPose().getX(), m_robotContainer.getSwerveDrivetrain().getPose().getY(), Rotation2d.k180deg));
    //}
    
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
        var alliance = DriverStation.getAlliance();

    Rotation2d autoRotation =
        alliance.isPresent() && alliance.get() == Alliance.Red
            ? Rotation2d.k180deg
            : Rotation2d.kZero;

    Pose2d startPose = new Pose2d(
        m_robotContainer.getSwerveDrivetrain().getPose().getTranslation(),
        autoRotation
    );

    m_robotContainer.getSwerveDrivetrain().resetPose(startPose);
    m_robotContainer.getSwerveDrivetrain().resetGyro(autoRotation.getDegrees());
    
    /*var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        m_robotContainer.getSwerveDrivetrain().resetGyro(180);
    } else {
        m_robotContainer.getSwerveDrivetrain().resetGyro(0);
    }
        */
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
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
    CommandScheduler.getInstance().schedule(new InstantCommand(()->{m_robotContainer.getHopperFeeder().setTopRollerSpeed(0);}));
  }

  @Override
  public void teleopInit() {
    m_robotContainer.getSwerveDrivetrain().resetRotation(Rotation2d.fromDegrees(degreesOutOfAuto));
    m_robotContainer.getSwerveDrivetrain().resetGyro(degreesOutOfAuto);

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
  public void teleopPeriodic() {}

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
