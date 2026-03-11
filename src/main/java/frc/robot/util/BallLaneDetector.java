package frc.robot.util;

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

            if (x < -10) {
                leftCount++;
            } 
            else if (x > 10) {
                rightCount++;
            } 
            else {
                middleCount++;
            }
        }

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