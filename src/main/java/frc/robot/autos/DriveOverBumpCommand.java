package frc.robot.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveOverBumpCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric driveRequest;
    private final Rotation2d targetHeading;

    private final PIDController headingPID;

    //climb states
    private enum State {
        APPROACH,
        CLIMB,
        DESCEND,
        FLAT_CONFIRM,
        DONE
    }

    private State currentState;

    private double flatStartTime = -1;

    // Speed values for states
    private final double approachSpeed = 0.5;
    private final double climbSpeed = 1.5;
    private final double descendSpeed = 1;
    private final double flatSpeed = 0.5;

    private final double climbThreshold = 7.0;
    private final double flatThreshold = 3.0;

    public DriveOverBumpCommand(
        CommandSwerveDrivetrain drivetrain,
        Rotation2d targetHeading,
        SwerveRequest.FieldCentric driveRequest
    ) {
        this.drivetrain = drivetrain;
        this.targetHeading = targetHeading;
        this.driveRequest = driveRequest;
        
        headingPID = new PIDController(4.0, 0.0, 0.15);

        headingPID.enableContinuousInput(-Math.PI, Math.PI);
        headingPID.setTolerance(Math.toRadians(2.0));

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        currentState = State.APPROACH;
        flatStartTime = -1;
        headingPID.reset();
    }

    @Override
    public void execute() {
        double pitch = drivetrain.getPigeon2().getPitch().getValueAsDouble();
        double roll  = drivetrain.getPigeon2().getRoll().getValueAsDouble();
        //TOTAL tILD
        double totalTilt = Math.sqrt(pitch * pitch + roll * roll);

        double driveSpeed = 0;

        switch (currentState) {

            case APPROACH:
                driveSpeed = approachSpeed;
                if (totalTilt > climbThreshold) {
                    currentState = State.CLIMB;
                }
                break;

            case CLIMB:
                driveSpeed = climbSpeed;
                if (totalTilt < climbThreshold) {
                    currentState = State.DESCEND;
                }
                break;

            case DESCEND:
                driveSpeed = descendSpeed;
                if (totalTilt < flatThreshold) {
                    currentState = State.FLAT_CONFIRM;
                    flatStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
                }
                break;

            case FLAT_CONFIRM:
                driveSpeed = flatSpeed;

                if (totalTilt > flatThreshold) {
                    // bounced back up, reset
                    currentState = State.DESCEND;
                } else {
                    double timeStayedFlat =
                        edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - flatStartTime;

                    if (timeStayedFlat > 0.25) {
                        currentState = State.DONE;
                    }
                }
                break;

            case DONE:
                driveSpeed = 0;
                break;
        }

        // --- Heading PID ---
        double omega =
            headingPID.calculate(
                drivetrain.getPose().getRotation().getRadians(),
                targetHeading.getRadians()
            );

        omega = MathUtil.clamp(omega, -3.0, 3.0);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(driveSpeed)
                .withVelocityY(0.0)
                .withRotationalRate(omega)
        );
    }

    @Override
    public boolean isFinished() {
        return currentState == State.DONE;    
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
        headingPID.reset();
    }
}

