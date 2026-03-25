package frc.robot.autos;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

public class CustomPathfinder implements Pathfinder {

    private final LocalADStar internal = new LocalADStar();

    @Override
    public boolean isNewPathAvailable() {
        return internal.isNewPathAvailable();
    }

    @Override
    public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
        return internal.getCurrentPath(constraints, goalEndState);
    }

    @Override
    public void setStartPosition(Translation2d startPosition) {
        internal.setStartPosition(startPosition);
    }

    @Override
    public void setGoalPosition(Translation2d goalPosition) {
        internal.setGoalPosition(goalPosition);
    }

    @Override
    public void setDynamicObstacles(
        List<Pair<Translation2d, Translation2d>> obs,
        Translation2d currentRobotPos
    ) {
        internal.setDynamicObstacles(obs, currentRobotPos);
    }
}