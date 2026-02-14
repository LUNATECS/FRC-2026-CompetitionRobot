// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterBangBangControlCommand extends Command {
  private BangBangController bangController;
  private ShooterSubSystem shooter; 
  private double targetRPM;
  private SimpleMotorFeedforward feedforward;
  private SlewRateLimiter voltageLimiter;
  /** Creates a new ShooterBangBangControlCommand. */
  public ShooterBangBangControlCommand(ShooterSubSystem shooter, double targetRPM) {
    this.shooter = shooter;
    this.targetRPM = targetRPM;
    bangController = new BangBangController(50);
    //WE WILL CHANGE THIS VALUE AFTER TESTING
    voltageLimiter = new SlewRateLimiter(100);
    //TUNE THIS DEPENDING ON HOW FAST YOU WANT IT TO RAMP UP. 
    //Basically, this is saying that it takes about 0.25 seconds to go to max voltage

    //FEEDFORWARD VALUES COMPUTED BY RECALC, except kS, which is completely new.
    feedforward = new SimpleMotorFeedforward(0.35, 0.17, 1.41);
    addRequirements(shooter);

    //FOR TUNING,START WITH FEEDFORWARD ONLY,
    //THEN ADD SLEW
    //THEN ADD BANG, until oscilation, then back off
    //then test ig.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    voltageLimiter.reset(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double bangControllerSpeed = bangController.calculate(shooter.getRPM(), RPM);
    //shooter.setRPM(bangControllerSpeed);
    //ABOVE STUFF IS WRONG

    //shooter.setVoltage(bangController.calculate(shooter.getRPM(), RPM) * 12.0 + 0.9 * feedforward.calculate(RPM));

    double bang = bangController.calculate(shooter.getRPM(), targetRPM);
    double feedForwardVolts = feedforward.calculate(targetRPM/60);
    double rawVolts = bang * 12 + feedForwardVolts;
    rawVolts = MathUtil.clamp(rawVolts, -12, 12);
    double smoothVolts = voltageLimiter.calculate(rawVolts);
    shooter.setVoltage(smoothVolts);

    //AFTER THIS WORKS, WE CAN INCORPORATE BOTH.
  SmartDashboard.putNumber("Shooter Target RPM Bang Bang", targetRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
