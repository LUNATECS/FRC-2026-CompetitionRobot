package frc.robot.subsystems;


import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightHelpers;
/**
* Class that extends the Phoenix 6 SwerveDrivetrain class and implements
* Subsystem so it can easily be used in command-based projects.
*/
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private Translation2d filteredVelocity = new Translation2d(0, 0);
    private final double alpha = 0.2;
   private static final double kSimLoopPeriod = 0.005; // 5 ms
   private Notifier m_simNotifier = null;
   private double m_lastSimTime;
   edu.wpi.first.wpilibj.AnalogInput sensor = new AnalogInput(3);
   /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
   private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
   /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
   private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
     /** Swerve request to apply during field-centric path following */
   private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
   private final PIDController m_pathXController = new PIDController(0, 0, 0);
   private final PIDController m_pathYController = new PIDController(0, 0, 0);
   private final PIDController m_pathThetaController = new PIDController(0, 0, 0);
   private Pose2d[] currentAlignmentSide = new Pose2d[]{new Pose2d(0,0, new Rotation2d()), new Pose2d(0,0, new Rotation2d())};
   /* Keep track if we've ever applied the operator perspective before or not */
   private boolean m_hasAppliedOperatorPerspective = false;
   private boolean isAligning = false;
   NetworkTable limelightShooter = NetworkTableInstance.getDefault().getTable("limelight-shooter");
   NetworkTable limelightHopper = NetworkTableInstance.getDefault().getTable("limelight-hopper");

   private final SwerveRequest.ApplyRobotSpeeds m_ApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
   private boolean needsVisionReset = false;
   private boolean isSkidding = false;
   private boolean isSpinning = false;
   private boolean doRejectUpdate = false;
   private boolean autoScore = false;
   private boolean isAligned = false;
   /* Swerve requests to apply during SysId characterization */
   private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
   private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
   private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
   private final SwerveDrivePoseEstimator pose = new SwerveDrivePoseEstimator(getKinematics(), getPigeon2().getRotation2d(), getModulePositions(), new Pose2d());
   /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
   private final NetworkTable robotPoseTable = NetworkTableInstance.getDefault().getTable("RobotPoseDashboard");
   private Pose2d targetPose = new Pose2d(4.625, 4.04, new Rotation2d(0)); // Example target pose on the field
   //targetPose ONLY for blue


   private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
       new SysIdRoutine.Config(
           null,        // Use default ramp rate (1 V/s)
           Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
           null,        // Use default timeout (10 s)
           // Log state with SignalLogger class
           state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
       ),
       new SysIdRoutine.Mechanism(
           output -> setControl(m_translationCharacterization.withVolts(output)),
           null,
           this
       )
   );


   /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
   @SuppressWarnings("unused")
   private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
       new SysIdRoutine.Config(
           null,        // Use default ramp rate (1 V/s)
           Volts.of(7), // Use dynamic voltage of 7 V
           null,        // Use default timeout (10 s)
           // Log state with SignalLogger class
           state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
       ),
       new SysIdRoutine.Mechanism(
           volts -> setControl(m_steerCharacterization.withVolts(volts)),
           null,
           this
       )
   );


   /*
    * SysId routine for characterizing rotation.
    * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
    * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
    */
   @SuppressWarnings("unused")
   private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
       new SysIdRoutine.Config(
           /* This is in radians per second², but SysId only supports "volts per second" */
           Volts.of(Math.PI / 6).per(Second),
           /* This is in radians per second, but SysId only supports "volts" */
           Volts.of(Math.PI),
           null, // Use default timeout (10 s)
           // Log state with SignalLogger class
           state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
       ),
       new SysIdRoutine.Mechanism(
           output -> {
               /* output is actually radians per second, but SysId only supports "volts" */
               setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
               /* also log the requested output for SysId */
               SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
           },
           null,
           this
       )
   );


   /* The SysId routine to test */
   private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
       private boolean otfFollowing;
  
       /**
        * Constructs a CTRE SwerveDrivetrain using the specified constants.
        * <p>
        * This constructs the underlying hardware devices, so users should not construct
        * the devices themselves. If they need the devices, they can access them through
        * getters in the classes.
        *
        * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
        * @param modules               Constants for each specific module
        */
       public CommandSwerveDrivetrain(
           SwerveDrivetrainConstants drivetrainConstants,
           SwerveModuleConstants<?, ?, ?>... modules
       ) {
           super(drivetrainConstants, modules);
           if (Utils.isSimulation()) {
               startSimThread();
           }
           configureAutoBuilder();
       }
  
       /**
        * Constructs a CTRE SwerveDrivetrain using the specified constants.
        * <p>
        * This constructs the underlying hardware devices, so users should not construct
        * the devices themselves. If they need the devices, they can access them through
        * getters in the classes.
        *
        * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
        * @param odometryUpdateFrequency The frequency to run the odometry loop. If
        *                                unspecified or set to 0 Hz, this is 250 Hz on
        *                                CAN FD, and 100 Hz on CAN 2.0.
        * @param modules                 Constants for each specific module
        */
       public CommandSwerveDrivetrain(
           SwerveDrivetrainConstants drivetrainConstants,
           double odometryUpdateFrequency,
           SwerveModuleConstants<?, ?, ?>... modules
       ) {
           super(drivetrainConstants, odometryUpdateFrequency, modules);
           if (Utils.isSimulation()) {
               startSimThread();
           }
           configureAutoBuilder();
       }
  
       /**
        * Constructs a CTRE SwerveDrivetrain using the specified constants.
        * <p>
        * This constructs the underlying hardware devices, so users should not construct
        * the devices themselves. If they need the devices, they can access them through
        * getters in the classes.
        *
        * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
        * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
        *                                  unspecified or set to 0 Hz, this is 250 Hz on
        *                                  CAN FD, and 100 Hz on CAN 2.0.
        * @param odometryStandardDeviation The standard deviation for odometry calculation
        *                                  in the form [x, y, theta]ᵀ, with units in meters
        *                                  and radians
        * @param visionStandardDeviation   The standard deviation for vision calculation
        *                                  in the form [x, y, theta]ᵀ, with units in meters
        *                                  and radians
        * @param modules                   Constants for each specific module
        */
       public CommandSwerveDrivetrain(
           SwerveDrivetrainConstants drivetrainConstants,
           double odometryUpdateFrequency,
           Matrix<N3, N1> odometryStandardDeviation,
           Matrix<N3, N1> visionStandardDeviation,
           SwerveModuleConstants<?, ?, ?>... modules
       ) {
           super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
           if (Utils.isSimulation()) {
               startSimThread();
           }
           configureAutoBuilder();
       }
       /**
    * Creates a new auto factory for this drivetrain.
    *
    * @return AutoFactory for this drivetrain
    */
   


 /**
  * Returns a command that applies the specified control request to this swerve drivetrain.
  *
  * @param request Function returning the request to apply
  * @return Command to run
  */
 public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
     return run(() -> this.setControl(requestSupplier.get()));
 }

 /**
  * Follows the given field-centric path sample with PID.
  *
  * @param sample Sample along the path to follow
  */
 
     
  
       /**
        * Runs the SysId Quasistatic test in the given direction for the routine
        * specified by {@link #m_sysIdRoutineToApply}.
        *
        * @param direction Direction of the SysId Quasistatic test
        * @return Command to run
        */
       public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
           return m_sysIdRoutineToApply.quasistatic(direction);
       }
  
       /**
        * Runs the SysId Dynamic test in the given direction for the routine
        * specified by {@link #m_sysIdRoutineToApply}.
        *
        * @param direction Direction of the SysId Dynamic test
        * @return Command to run
        */
       public Command sysIdDynamic(SysIdRoutine.Direction direction) {
           return m_sysIdRoutineToApply.dynamic(direction);
       }
  
       @Override
       public void periodic() {

        SmartDashboard.putNumber("drictrain distance to target", pose.getEstimatedPosition().getTranslation()
                                .getDistance(targetPose.getTranslation()));
        //Setting Rotations
        double robotYaw = getPigeon2().getYaw().getValueAsDouble();
        double robotPitch = getPigeon2().getPitch().getValueAsDouble();
        double robotRoll = getPigeon2().getRoll().getValueAsDouble();
        SmartDashboard.putNumber("pidgeon2 reading", robotYaw);
        SmartDashboard.putNumber("pidgeon roll", robotRoll);
        SmartDashboard.putNumber("pidgeon pitch", robotPitch);
        SmartDashboard.putNumber("pidgeon aceleration x ", getPigeon2().getAccelerationX().getValueAsDouble()); 
        SmartDashboard.putNumber("pidgeon aceleration y ", getPigeon2().getAccelerationY().getValueAsDouble()); 

        Translation2d rawVel = getFieldVelocity();
            filteredVelocity = new Translation2d(
                filteredVelocity.getX() + alpha * (rawVel.getX()-filteredVelocity.getX()),
                filteredVelocity.getY() + alpha * (rawVel.getY() - filteredVelocity .getY())
            );
        
        SmartDashboard.putNumber("Filtered Velocity X", filteredVelocity.getX());
        SmartDashboard.putNumber("Filtered Velocity Y", filteredVelocity.getY());
        SmartDashboard.putNumber("Filtered Velocity Magnitude", filteredVelocity.getNorm());

        //Double Limelight Orientations
        LimelightHelpers.SetRobotOrientation("limelight-shooter", robotYaw, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight-hopper", robotYaw, 0, 0, 0, 0, 0);

        //update odometry
        pose.update(getPigeon2().getRotation2d(), getModulePositions());

        //getting LL measurements
        LimelightHelpers.PoseEstimate frontMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-shooter");
        LimelightHelpers.PoseEstimate backMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-hopper");

        //boolean useFront = false;
        //boolean useBack = false;

        //only use camera if it sees atleast one tag
        /*if (frontMeasurement != null && frontMeasurement.tagCount > 0) {
        useFront = true;
        }
        if (backMeasurement != null && backMeasurement.tagCount > 0) {
            useBack = true;
        }

        //pick the one with more tags
        if (useFront && useBack) {
            if (frontMeasurement.tagCount >= backMeasurement.tagCount) {
                useBack = false; // prefer front
            } else {
                useFront = false; // prefer back
            }
        }
        //reject spinning too fast
        double angularVelocityZ = getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
        if (Math.abs(angularVelocityZ) > 720) {
            useFront = false;
            useBack = false;
        }


        //apply vision measurements
        if (useFront) {
          pose.addVisionMeasurement(frontMeasurement.pose, frontMeasurement.timestampSeconds);
        //pose.addVisionMeasurement(
          //  frontMeasurement.pose,
          //  frontMeasurement.timestampSeconds,
          //  VecBuilder.fill(0.7, 0.7, 9999999) // adjust confidence weights if needed
        //);
        } else if (useBack) {
            pose.addVisionMeasurement(backMeasurement.pose, backMeasurement.timestampSeconds);

            //pose.addVisionMeasurement(
              //  backMeasurement.pose,
              //  backMeasurement.timestampSeconds,
              //  VecBuilder.fill(0.7, 0.7, 9999999)
            //);
        }
            */
        //ABOVE IS FOR ONLY USING ONE LIMELIGHT APPROACH

        //BELOW IS FUSING BOTH, but it's commented out for now :(
        // Reject vision updates if spinning too fast
         
        double angularVelocityZ = getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
        boolean rejectVision = Math.abs(angularVelocityZ) > 720;

        if (!rejectVision) {
            // Default standard deviations
            double frontWeight = 0.7; // smaller = more confidence and more integrated
            double backWeight = 0.7;

            // Increase confidence if one camera sees more tags than the other
            if (frontMeasurement != null && backMeasurement != null) {
                if (frontMeasurement.tagCount > backMeasurement.tagCount) {
                    frontWeight = 0.4; // more trusted
                    backWeight = 1.0;  // less trusted
                } else if (backMeasurement.tagCount > frontMeasurement.tagCount) {
                    backWeight = 0.4;  // more trusted
                    frontWeight = 1.0; // less trusted
                }
                // if equal, keep default
            }

            // Fuse front Limelight
            if (frontMeasurement != null && frontMeasurement.tagCount > 0) {
                pose.addVisionMeasurement(
                    frontMeasurement.pose,
                    frontMeasurement.timestampSeconds,
                    VecBuilder.fill(frontWeight, frontWeight, 9999999)
                );
            }

            // Fuse back Limelight
            if (backMeasurement != null && backMeasurement.tagCount > 0) {
                pose.addVisionMeasurement(
                    backMeasurement.pose,
                    backMeasurement.timestampSeconds,
                    VecBuilder.fill(backWeight, backWeight, 9999999)
                );
            }
        }
        



        //OLD VERSION-Single Limelight
        /* 
         double robotYaw = getPigeon2().getYaw().getValueAsDouble();
         double robotPitch = getPigeon2().getPitch().getValueAsDouble();
         double robotRoll = getPigeon2().getRoll().getValueAsDouble();
         
         //Use roll and pitch values if we want to actually see if we're over the bump or not
         LimelightHelpers.SetRobotOrientation("limelight-front", robotYaw, 0, 0, 0, 0, 0);


         pose.update(getPigeon2().getRotation2d(), getModulePositions());
         SmartDashboard.putNumber("Swerve Robot Pose", getState().Pose.getRotation().getDegrees());
         // // SmartDashboard.putBoolean("Range valid", distanceSensor.isRangeValid());
         // // SmartDashboard.putNumber("Distance sensed", getSensorVal());
         LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
             if(Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
         {
           doRejectUpdate = true;
         }
         if(limelightMeasurement.tagCount == 0)
         {
           doRejectUpdate = true;
         }
         if(!doRejectUpdate)
         {
           pose.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
           pose.addVisionMeasurement(
               limelightMeasurement.pose,
               limelightMeasurement.timestampSeconds);
         }
      
         //if (getTVLeft()) {
           //var driveState = this.getState();
           //double headingDeg = driveState.Pose.getRotation().getDegrees();
           //SmartDashboard.putNumber("Current Heading DEG", headingDeg);
           //double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
    
           //LimelightHelpers.SetRobotOrientation("limelight-left", headingDeg, 0, 0, 0, 0, 0);
           //var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
           //if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
            // pose.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
           //}
         //}
         // if our angular velocity is greater than 360 degrees per second, ignore vision updates
    
         //   // SmartDashboard.putNumber("Rot", getPose().getRotation().getDegrees());
           /*
            * Periodically try to apply the operator perspective.
            * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
            * This allows us to correct the perspective in case the robot code restarts mid-match.
            * Otherwise, only check and apply the operator perspective if the DS is disabled.
            * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
            */
           if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
               DriverStation.getAlliance().ifPresent(allianceColor -> {
                   setOperatorPerspectiveForward(
                       allianceColor == Alliance.Red
                           ? kRedAlliancePerspectiveRotation
                           : kBlueAlliancePerspectiveRotation
                   );
                   m_hasAppliedOperatorPerspective = true;
               });
           }
           SmartDashboard.putString("Robot Pose", pose.toString());


           
        SmartDashboard.putString("Swerve Robot Pose", getState().Pose.toString());


             SmartDashboard.putString("Robot Pose Total", "X:" + getPose().getX() +
           " Y:" + getPose().getY() +
          " R:" + getPose().getRotation().getDegrees());
          Pose2d currentPose = getPose();
            
       }
  
       private void startSimThread() {
           m_lastSimTime = Utils.getCurrentTimeSeconds();
  
           /* Run simulation at a faster rate so PID gains behave more reasonably */
           m_simNotifier = new Notifier(() -> {
               final double currentTime = Utils.getCurrentTimeSeconds();
               double deltaTime = currentTime - m_lastSimTime;
               m_lastSimTime = currentTime;
  
               /* use the measured time delta, get battery voltage from WPILib */
               updateSimState(deltaTime, RobotController.getBatteryVoltage());
           });
           m_simNotifier.startPeriodic(kSimLoopPeriod);
       }
  
       /*-------------------------------------------------------------------------------------
         -------------------------------------------------------------------------------------
         -------------------------------------------------------------------------------------
         */
         public SwerveModulePosition[] getModulePositions() {
           return getState().ModulePositions;
         }
         public Pose2d getPose() {
           return pose.getEstimatedPosition();
         }
         public void resetPose(Pose2d rpose) {
           pose.resetPose(rpose);
           this.seedFieldCentric(rpose.getRotation());
         }

         public Rotation2d getRotationToFacePoint(Translation2d fieldTarget) {
                Pose2d robotPose = getPose(); // already fused pose estimator
                Translation2d robotToTarget =
                    fieldTarget.minus(robotPose.getTranslation());
                //double Angle = robotToTarget.getAngle().getDegrees() - 180;
                //Rotation2d finalRotateAngle = new Rotation2d(Angle);
                return robotToTarget.getAngle().plus(Rotation2d.fromDegrees(180));
        }

        public static double calculateProjectileSpeed(double wheelRPM) {
          //this whole method may need to be redone
            double wheelRadiusInches = 4.0625 / 2.0; 
            double wheelRadius = wheelRadiusInches * 0.0254; // convert inches to meters, since literally everythign else is in meters per sec
            double omegaRadPerSec = 2 * Math.PI * wheelRPM / 60.0; // convert RPM to rad/
            return omegaRadPerSec * wheelRadius * 0.95; //adjust slip factor depending on tuning
        }

        public Translation2d getFieldAcceleration() {
          double ax = getPigeon2().getAccelerationX().getValueAsDouble() * 9.80665;
          double ay = getPigeon2().getAccelerationY().getValueAsDouble() * 9.80665;

          Rotation2d rot = getPose().getRotation();

          return new Translation2d(ax, ay).rotateBy(rot);
      }

      public double computeFlightTime(double v, double deltaX, double deltaY, double launchAngleRad) {
        double g = 9.81; //gravivty
        double vy = v * Math.sin(launchAngleRad); //vertical component of velocity
        double discriminant = vy*vy - 2 * g * deltaY;
        if(discriminant < 0){
           return deltaX / (v * Math.cos(launchAngleRad)); // fallback
        }
        double t1 = (vy + Math.sqrt(discriminant)) / g;
        double t2 = (vy - Math.sqrt(discriminant)) / g;
        return Math.max(t1, t2); //returns the two possible flight times, but it chooses the max to see when it reaches target on the way down
    }

      public Translation2d getFieldVelocity() {
        var speeds = getState().Speeds; // vx, vy in robot frame (m/s)
        Rotation2d heading = getPigeon2().getRotation2d();

        // Rotate robot-relative velocity into field coordinates
        double vxField = speeds.vxMetersPerSecond * heading.getCos() - speeds.vyMetersPerSecond * heading.getSin();
        double vyField = speeds.vxMetersPerSecond * heading.getSin() + speeds.vyMetersPerSecond * heading.getCos();
        
        return new Translation2d(vxField, vyField);
      }




         
         public void resetGyro(double angle) {
           getPigeon2().setYaw(angle);
         }
         public void setShouldAutoScore() {
           autoScore = true;
         }
         public void turnOffAutoScore() {
           autoScore = false;
         }
         public boolean getAutoScoreVal() {
           return autoScore;
         }
         public double getTXShooter() {
           return limelightShooter.getEntry("tx").getDouble(0.);
         }
         public double getTXHopper() {
           return limelightHopper.getEntry("tx").getDouble(0.);
         }

         public double getTYShooter() {
           return limelightShooter.getEntry("ty").getDouble(0.);
         }
         public double getTYHopper() {
           return limelightHopper.getEntry("ty").getDouble(0.);
         }
         public void resetPoseBasedOnShooterLL() {
           var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-shooter");
           if (llMeasurement != null && llMeasurement.tagCount > 0) {
             pose.resetPose(llMeasurement.pose);
           }
         }
         public void resetPoseBasedOnHopperLL() {
           var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-hopper");
           if (llMeasurement != null && llMeasurement.tagCount > 0) {
             pose.resetPose(llMeasurement.pose);
           }
         }


         public Command setFollowingPath() {
           return new InstantCommand(() -> otfFollowing = true);
     }
     public Command stopPathFollowState() {
       return new InstantCommand(() -> otfFollowing = false);
     }
   //   public boolean getTV() {
   //     return m_limelight.getEntry("tv").getDouble(0.0) == 1.0;
   //   }
     public void setAligning(boolean aligning) {
       isAligning = aligning;
     }
     public boolean isAligning() {
       return isAligning;
     }
    
   //   public double getTZ() {
   //     return m_limelight.getEntry("ty").getDouble(0.0);
   //   }
   //   public Pose2d getPoseLL() {
   //     var array = m_limelight.getEntry("botpose_wpired").getDoubleArray(new double[]{});
   //     double[] result = {array[0], array[1], array[5]};
   //     Pose2d pose = new Pose2d(result[0], result[1], new Rotation2d(result[2]));
   //     return pose;
   //     // double[] poseArray = {pose.getX(), pose.getY(), ((pose.getRotation().getDegrees())/360)+(pose.getRotation().getDegrees()%360)};
   //     // table.getEntry("RobotPose").setDoubleArray(poseArray);
   //     // SmartDashboard.putNumberArray("Raw Pose", result);
   //   }
     public Pose2d getShooterLLPose() {
       var array = limelightShooter.getEntry("botpose_wpiblue").getDoubleArray(new double[]{0,0,0,0,0,0});
       double[] result = {array[0], array[1], array[5]};
       Pose2d pose = new Pose2d(result[0], result[1], new Rotation2d(result[2]));
       // return pose;
       return pose;
     }
     public Pose2d getHopperLLPose() {
       var array = limelightHopper.getEntry("botpose_wpiblue").getDoubleArray(new double[]{0,0,0,0,0,0});
       double[] result = {array[0], array[1], array[5]};
       Pose2d pose = new Pose2d(result[0], result[1], new Rotation2d(result[2]));
       // return pose;
       return pose;
     }
     private void configureAutoBuilder() {
       try {
           var config = RobotConfig.fromGUISettings();
           AutoBuilder.configure(() -> getPose(),
                               this::resetPose,
                               () -> getState().Speeds,
                               (speeds, feedforwards) -> setControl(
                                   m_ApplyRobotSpeeds.withSpeeds(speeds)
                       .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                       .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                               ),
                               new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                               new PIDConstants(1.75, 0.0, 0.0), // Translation PID constants
                               new PIDConstants(4., 0.0, 0.0)
                               //HAVE TO ADJUST THESE PID CONFIGS
           ),
                               config,
                               () -> {
                                   // Boolean supplier that controls when the path will be mirrored for the red alliance
                                   // This will flip the path being followed to the red side of the field.
                                   // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    
                                   var alliance = DriverStation.getAlliance();
                                   if (alliance.isPresent()) {
                                     return alliance.get() == DriverStation.Alliance.Red;
                                   }
                                   return false;
                                 },
                                   this);


                                  
               }
               catch (Exception ex) {
                   DriverStation.reportError("Error", ex.getStackTrace());
               }
       }
   /**
    * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
    * while still accounting for measurement noise.
    *
    * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
    * @param timestampSeconds The timestamp of the vision measurement in seconds.
    */
   @Override
   public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
       super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
   }


   /**
    * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
    * while still accounting for measurement noise.
    * <p>
    * Note that the vision measurement standard deviations passed into this method
    * will continue to apply to future measurements until a subsequent call to
    * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
    *
    * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
    * @param timestampSeconds The timestamp of the vision measurement in seconds.
    * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
    *     in the form [x, y, theta]ᵀ, with units in meters and radians.
    */
   @Override
   public void addVisionMeasurement(
       Pose2d visionRobotPoseMeters,
       double timestampSeconds,
       Matrix<N3, N1> visionMeasurementStdDevs
   ) {
       super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
   }
   public void checkForSkid() {
     TalonFX[] driveMotors = new TalonFX[4];
     driveMotors[0] = getModules()[0].getDriveMotor();
     driveMotors[1] = getModules()[1].getDriveMotor();
     driveMotors[2] = getModules()[2].getDriveMotor();
     driveMotors[3] = getModules()[3].getDriveMotor();
     var averageCurrent = 0;
     for (int i=0; i<3; i++) {
       averageCurrent += driveMotors[i].getStatorCurrent().getValueAsDouble();
     }
     averageCurrent /= 4;
     if (averageCurrent > 60) {
       isSkidding = true;
     }
     else {
       isSkidding = false;
     }
   }
}

