// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShootOnTheMove.FullShootOnTheMoveSequence;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubSystem;
import frc.robot.subsystems.HopperFeederSubSystem;
import frc.robot.subsystems.HopperSubSystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterFeederSubSystem;
import frc.robot.subsystems.ShooterSubSystem;

/** Add your docs here. */
public class AutoFactory {

    private static final Map<String, PathPlannerPath> pathCache = new HashMap<>();

    private static PathPlannerPath getPath(String name) {
        return pathCache.computeIfAbsent(name, n -> {
            try {
                System.out.println("Loading path: " + n);
                return PathPlannerPath.fromPathFile(n);
            } catch (Exception e) {  // catches ALL 3 exceptions
                System.err.println("FAILED to load path: " + n);
                e.printStackTrace();
                return null;
            }
        });
    }


    public static void preloadPaths(String... names) {
    for (String name : names) {
        getPath(name); // this will cache it
        }
    }

    public static void preloadPaths() {
    preloadPaths(
        "Trench Left P1",
        "Trench Left SOTM Path",
        "Trench Left SOTM P2",
        "Trench Right P1",
        "Trench Right SOTM Path",
        "Trench Right SOTM P2",
        "Trench Right SOTM P2 WIDE",
        "Trench Left SOTM P2 WIDE"
        // add ALL paths used in autos here
    );
}



    //EVERYTING HAS TO BE TUNED

    private static final PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
    
        public static Command pathFindThenFollowPath(String name) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(name);
            return AutoBuilder.pathfindThenFollowPath(path, constraints);
        } catch(Exception e) {
            e.printStackTrace();
            return Commands.none();
        }
    }

        


        public static Command followPath(String name) {
            try {
                PathPlannerPath path = getPath(name);
                return AutoBuilder.followPath(path);
            }
            catch(Exception e) {
                e.printStackTrace();
                return Commands.none();
            }
        }

        private static final PathConstraints intakeBallsConstraints = new PathConstraints(
        1.5, 1.5,
        Units.degreesToRadians(540), Units.degreesToRadians(720));


        public static Command pathfindToPose(Pose2d pose) {
            return AutoBuilder.pathfindToPose(pose, intakeBallsConstraints);
        }


        public static Supplier<Translation2d> getPathTranslationSupplier(String pathName) {
            try {
                // load the path
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

                // get the trajectory from the path using config
                var config = RobotConfig.fromGUISettings();
                java.util.Optional<PathPlannerTrajectory> optionalTraj =
                    path.getIdealTrajectory(config);

                if (optionalTraj.isEmpty()) {
                    System.err.println("Failed to get trajectory for path: " + pathName);
                    return () -> new Translation2d(0, 0);
                }

                PathPlannerTrajectory trajectory = optionalTraj.get();
                final double startTime = Timer.getFPGATimestamp();

                return () -> {
                    double time = Timer.getFPGATimestamp() - startTime;

                    // sample the trajectory at the current time
                    PathPlannerTrajectoryState state = trajectory.sample(time);

                    // use fieldSpeeds (vx, vy) instead of velocityMetersPerSecond
                    double vx = state.fieldSpeeds.vxMetersPerSecond;
                    double vy = state.fieldSpeeds.vyMetersPerSecond;

                    return new Translation2d(vx, vy);
                };
            } catch (Exception e) {
                e.printStackTrace();
                return () -> new Translation2d(0, 0);
            }
        }  


                public static Command makeShootOnTheMovePathCommand(
                CommandSwerveDrivetrain drivetrain,
                String pathName,
                Supplier<Translation2d> targetSupplier,
                SwerveRequest.FieldCentric driveRequest,
                ShooterSubSystem shooter,
                ShooterFeederSubSystem shooterFeeder,
                HopperFeederSubSystem hopperFeeder,
                HopperSubSystem hopper,
                HoodSubSystem hood,
                IntakeSubSystem intake,
                LEDSubsystem led
        ) {
            try {
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                var config = RobotConfig.fromGUISettings();
                var optionalTraj = path.getIdealTrajectory(config);

                if (optionalTraj.isEmpty()) {
                    System.err.println("Failed to get trajectory for path: " + pathName);
                    return Commands.none();
                }

                PathPlannerTrajectory trajectory = optionalTraj.get();
                final double startTime = Timer.getFPGATimestamp();
                final double totalTime = trajectory.getTotalTimeSeconds();

                // Use a supplier for translational velocities
                Supplier<Translation2d> translationSupplier = () -> {
                    double time = Timer.getFPGATimestamp() - startTime;
                    if (time > totalTime) time = totalTime;
                    var state = trajectory.sample(time);
                    return new Translation2d(state.fieldSpeeds.vxMetersPerSecond, state.fieldSpeeds.vyMetersPerSecond);
                };

                // Build your SOTM sequence command using the translation supplier
                FullShootOnTheMoveSequence sotm = new FullShootOnTheMoveSequence(
                    drivetrain,
                    shooter,
                    hood,
                    hopperFeeder,
                    shooterFeeder,
                    hopper,
                    intake,
                    led,
                    translationSupplier,
                    targetSupplier,
                    driveRequest,
                    ()->true
                );

                // Wrap in a command that ends when the path ends
                return Commands.sequence(
                    sotm.withTimeout(totalTime)
                );

            } catch (Exception e) {
                e.printStackTrace();
                return Commands.none();
            }
        }
}
