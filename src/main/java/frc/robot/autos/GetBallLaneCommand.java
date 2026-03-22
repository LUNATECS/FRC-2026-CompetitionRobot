
package frc.robot.autos;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.BallLaneDetector;

public class GetBallLaneCommand extends Command {

    private double startTime;
    private String result;
    private final CommandSwerveDrivetrain drivetrain;

    public GetBallLaneCommand(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        drivetrain.setHopperLimelightPipeline(1);
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (result == null && Timer.getFPGATimestamp() - startTime > 0.1) {
        result = BallLaneDetector.getBestLane("limelight-hopper");
        }
    }

   @Override
    public boolean isFinished() {
        return result != null || Timer.getFPGATimestamp() - startTime > 0.5;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setHopperLimelightPipeline(0);
        SmartDashboard.putString("AUTO DETECTED LANE", result);
    }

    public String getResult() {
        return result;
    }
}