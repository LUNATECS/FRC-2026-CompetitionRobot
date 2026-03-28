package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.LimelightHelpers.RawDetection;

public class BallLaneDetector {

    public static String getBestLane(String limelightName) {

        RawDetection[] detections = LimelightHelpers.getRawDetections(limelightName);

        int leftCount = 0;
        int middleCount = 0;
        int rightCount = 0;

        for (RawDetection detection : detections) {

            double x = detection.txnc; // horizontal position of the ball

            if (x < -11) {
                leftCount++;
            } 
            else if (x > 11) {
                rightCount++;
            } 
            else {
                middleCount++;
            }
        }
        SmartDashboard.putNumber("LL Left Count", leftCount);
        SmartDashboard.putNumber("LL Middle Count", middleCount);
        SmartDashboard.putNumber("LL Right Count", rightCount);

        if (leftCount >= middleCount && leftCount >= rightCount) {
            return "left";
        } 
        else if (rightCount >= leftCount && rightCount >= middleCount) {
            return "right";
        } 
        else {
            return "middle";
        }
    }
}