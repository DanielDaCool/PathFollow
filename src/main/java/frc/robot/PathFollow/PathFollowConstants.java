
package frc.robot.PathFollow;

import edu.wpi.first.math.controller.PIDController;

public class PathFollowConstants {
    public static PIDController ROTATION_PID = new PIDController(0.05,0.0, 0.002);
    public static double ANGLE_OFFSET = 3;
    public static double distanceOffset = 0.01;
}
