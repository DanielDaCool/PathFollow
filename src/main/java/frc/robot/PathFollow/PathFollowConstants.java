
package frc.robot.PathFollow;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.PathFollow.Util.Segment;

public class PathFollowConstants {
    public static double ROBOT_LENGTH = 0.50; //TODO
    public static double PATH_ANGLE_OFFSET = 3;
    public static double PATH_DISTANCE_OFFSET = 0.01;
    public static double PATH_MAX_VELOCITY = 3;
    public static double PATH_ACCEL = 10;
    public static double PATH_ROTATION_MAX_VELOCITY = Math.PI;
    public static double PATH_ROTATION_ACCEL = Math.PI * 2;
    public static double PATH_MIN_DISTANCE_FROM_CORNER = (ROBOT_LENGTH / 2) + 0.6; //in CM
    public static double PATH_MAX_VELOCITY_AVOID = 1.5;
    public static double PATH_MIN_DISTANCE_SEGMENT = 0.15;
    public static double FIELD_LENGTH = 16.54; // in meters
    public static double FIELD_HEIGHT = 8.21; // in meters

    public static double convertAlliance(double x) {
        return FIELD_LENGTH - x;
    }

    public static boolean isInPoint(Translation2d pos, Translation2d point){
        return pos.getDistance(point) <= 0.05;
    }

 

}