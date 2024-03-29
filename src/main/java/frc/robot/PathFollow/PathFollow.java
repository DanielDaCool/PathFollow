package frc.robot.PathFollow;

import static frc.robot.PathFollow.PathFollowConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.PathFollow.Util.AvoidBannedZone;
import frc.robot.PathFollow.Util.Leg;
import frc.robot.PathFollow.Util.RoundedPoint;
import frc.robot.PathFollow.Util.Segment;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.utils.Trapezoid;

public class PathFollow extends Command {
  Timer timer = new Timer();
  Chassis chassis;
  RoundedPoint[] corners;

  Pose2d chassisPose = new Pose2d();

  double pathLength;

  double totalLeft;
  int segmentIndex;

  Segment[] segments;
  Translation2d vecVel;
  Rotation2d wantedAngle;

  double distancePassed;
  double driveVelocity;
  double rotationVelocity;

  static double fieldLength = 16.54; // in meters
  static double fieldHeight = 8.21; // in meters
  
  boolean isRed;

  pathPoint[] points;

  double finishVel;
  double maxVel;
  double accel;

  Trapezoid driveTrapezoid;


  /**
   * Creates a new path follower using the given points.
   * 
   * @param chassis
   * @param points   from blue alliance
   * @param maxVel   the max velocity in m/s
   * @param maxAccel the max accel in m/s2 (squared)
   * 
   */


  public PathFollow(pathPoint[] points){
    this(RobotContainer.robotContainer.chassis, points, PATH_MAX_VELOCITY, PATH_ACCEL, 0);
    addRequirements(chassis);
  }

  public PathFollow(pathPoint[] points, double vel){
    this(RobotContainer.robotContainer.chassis, points, vel, PATH_ACCEL, 0);
    addRequirements(chassis);
  }
  public PathFollow(pathPoint[] points, double vel, double finishVel){
    this(RobotContainer.robotContainer.chassis, points, vel, PATH_ACCEL, finishVel);
    addRequirements(chassis);
  }

  public PathFollow(Chassis chassis, pathPoint[] points, double maxVel, double accel, double finishVel) {
    this.points = points;
    this.finishVel = finishVel;
    this.chassis = chassis;
    this.maxVel = maxVel;
    this.accel = accel; 
    addRequirements(chassis);

    

  }



    public static double convertAlliance(double x) {
    return fieldLength - x;
  }

  /*
   * public String currentSegmentInfo() {
   * 
   * if (segments == null)
   * return "";
   * return segments[segmentIndex].toString();
   * }
   */

  @Override
  public void initialize() {

    distancePassed = 0;

    
    driveTrapezoid = new Trapezoid(points[0].getVelocity(), PATH_ACCEL, points[1].getVelocity());

    segments = new Segment[1 + ((points.length - 2) * 2)];

    isRed = RobotContainer.robotContainer.isRed();
    // sets first point to chassis pose to prevent bugs with red and blue alliance
    points[0] = new pathPoint(chassis.getPose().getX(), chassis.getPose().getY(), points[1].getRotation(),
        points[0].getRadius(), chassis.getVelocity().getNorm());

    // case for red alliance (blue is the default)
    if (isRed) {
      points[0] = new pathPoint(chassis.getPose().getX(), chassis.getPose().getY(),
          Rotation2d.fromDegrees(180).minus(points[1].getRotation()), points[0].getRadius(), chassis.getVelocity().getNorm());
      for (int i = 1; i < points.length; i++) {
        points[i] = new pathPoint(fieldLength - points[i].getX(), points[i].getY(),
            Rotation2d.fromDegrees(180).minus(points[i].getRotation()),
            points[i].getRadius(), points[i].getVelocity());
            
      }
    }
    corners = new RoundedPoint[points.length - 2];
    for (int i = 0; i < points.length - 2; i++) {
      corners[i] = new RoundedPoint(points[i], points[i + 1], points[i + 2]);
    }
    // case for 1 segment, need to create only 1 leg
    if (points.length < 3) {
      segments[0] = AvoidBannedZone.fixPoint(new Leg(points[0].getTranslation(), points[1].getTranslation()), points[0].getTranslation());
      // System.out.println("------LESS THAN 3------");
    }
    // case for more then 1 segment
    else {
      // creates the first leg
      segments[0] = corners[0].getAtoCurveLeg();

      int segmentIndexCreator = 1;
      // creates arc than leg
      for (int i = 0; i < corners.length - 1; i += 1) {

        segments[segmentIndexCreator] = corners[i].getArc();

        segments[segmentIndexCreator + 1] = new Leg(corners[i].getCurveEnd(), corners[i + 1].getCurveStart());
        segmentIndexCreator += 2;
      }
      // creates the last arc and leg
      segments[segments.length - 2] = corners[corners.length - 1].getArc();
      segments[segments.length - 1] = corners[corners.length - 1].getCtoCurveLeg();
    }

    // calculates the length of the entire path
    double segmentSum = 0;
    for (Segment s : segments) {
      segmentSum += s.getLength();
    }
    pathLength = segmentSum;
    totalLeft = pathLength;
    segmentIndex = 0;


    vecVel = new Translation2d(0, 0);
  }

  // calculates the position of the closet april tag and returns it's position
 
  public static double fixY(double y) {
    return fieldHeight - y;
  }

  public boolean isFinishedSegment(double distancePassed, double segmentLength){
    return distancePassed >= segmentLength;
  }
  public boolean isLastSegment(int index){
    return index == segments.length - 1;
  }

  @Override
  public void execute() {
    chassisPose = chassis.getPose();

    // current velocity vector
    Translation2d currentVelocity = new Translation2d(chassis.getChassisSpeeds().vxMetersPerSecond, chassis.getChassisSpeeds().vyMetersPerSecond);

    //calc for distance passed based on total left minus distance passed on current segment
    distancePassed = totalLeft - segments[segmentIndex].distancePassed(chassisPose.getTranslation());

    wantedAngle = points[segmentIndex].getRotation();

    //update total left when finished current segment
    if(isFinishedSegment(segments[segmentIndex].distancePassed(chassisPose.getTranslation()), segments[segmentIndex].getLength() - PATH_DISTANCE_OFFSET)){
      totalLeft -= segments[segmentIndex].getLength();
      
      //update segment index
      if (!isLastSegment(segmentIndex) || segments[segmentIndex].getLength() <= 0.15){
        segmentIndex++;
        driveTrapezoid = new Trapezoid(points[segmentIndex].getVelocity(), accel, points[segmentIndex+1].getVelocity());
      }
      else driveTrapezoid = new Trapezoid(points[segmentIndex].getVelocity(), accel, finishVel);
    }

    //calc drive velocity
    driveVelocity = driveTrapezoid.calcVelocity(
        totalLeft - segments[segmentIndex].distancePassed(chassisPose.getTranslation()),
        currentVelocity.getNorm());

    Translation2d velVector = segments[segmentIndex].calc(chassisPose.getTranslation(), driveVelocity);

    
    //case for correct Translation2d but wrong angle so stop chassis but keep rotation
    if (totalLeft <= PATH_DISTANCE_OFFSET) velVector = new Translation2d(0, 0);

    //calc rotation velocity based on PID
    double rotationVelocity = (Math.abs(wantedAngle.minus(chassis.getAngle()).getDegrees()) <= PATH_ANGLE_OFFSET)
      ? 0 : PATH_ROTATION_PID.calculate(chassis.getAngle().getDegrees(), 0);

    ChassisSpeeds speed = new ChassisSpeeds(velVector.getX(), velVector.getY(), rotationVelocity); 
    chassis.setVelocities(speed);

  }
  @Override
  public void end(boolean interrupted) {
    if(finishVel == 0) chassis.stop();
  }

  @Override
  public boolean isFinished() {
    return totalLeft <= PATH_DISTANCE_OFFSET;
  }
}
