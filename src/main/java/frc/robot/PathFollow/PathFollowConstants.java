
package frc.robot.PathFollow;

import edu.wpi.first.math.controller.PIDController;

public class PathFollowConstants {
    public static PIDController PATH_ROTATION_PID = new PIDController(0.05,0.0, 0.002);
    public static double PATH_ANGLE_OFFSET = 3;
    public static double PATH_DISTANCE_OFFSET = 0.01;
    public static double PATH_MAX_VELOCITY = 3;
    public static double PATH_ACCEL = 10;
}
