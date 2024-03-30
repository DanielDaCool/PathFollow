package frc.robot.PathFollow;

import static frc.robot.PathFollow.PathFollowConstants.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.PathFollow.Util.AvoidBannedZone;
import frc.robot.PathFollow.Util.Leg;
import frc.robot.PathFollow.Util.RoundedPoint;
import frc.robot.PathFollow.Util.Segment;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.utils.Trapezoid;

public class PathFollow extends CommandBase {
  Timer timer = new Timer();
  Chassis chassis;
  RoundedPoint[] corners;

  Pose2d chassisPose = new Pose2d();

  double pathLength;

  double totalLeft;
  int segmentIndex;
  Trapezoid rotationTrapezoid;

  List<Segment> segments = new ArrayList<Segment>();
  HashMap<Segment, Rotation2d> anglePerSegment = new HashMap<Segment, Rotation2d>();
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

  
  public String currentSegmentInfo() {
    
    if (segments == null) return "";
    return segments.get(segmentIndex).toString();
  }
  private void initPoints(){
    
    isRed = RobotContainer.robotContainer.isRed();
    // sets first point to chassis pose to prevent bugs with red and blue alliance
    points[0] = new pathPoint(chassis.getPose().getX(), chassis.getPose().getY(), points[1].getRotation(),
        points[0].getRadius(), chassis.getVelocity().getNorm());

    // case for red alliance (blue is the default)
    if (isRed) {
      points[0] = new pathPoint(chassis.getPose().getX(), chassis.getPose().getY(),
          Rotation2d.fromDegrees(180).minus(points[1].getRotation()), points[0].getRadius(), chassis.getVelocity().getNorm());
      for (int i = 1; i < points.length; i++) {
        points[i] = new pathPoint(convertAlliance(points[i].getX()), points[i].getY(),
            Rotation2d.fromDegrees(180).minus(points[i].getRotation()),
            points[i].getRadius(), points[i].getVelocity());
            
      }
    }
    
  }
  
  private void initCorners(){
    corners = new RoundedPoint[points.length - 2];
    for (int i = 0; i < points.length - 2; i++) {
      corners[i] = new RoundedPoint(points[i], points[i + 1], points[i + 2]);
    }

  }

  private void initSegments(){
  if (points.length < 3) {
    for(Segment segment : AvoidBannedZone.fixPoint(new Leg(points[0].getTranslation(), points[1].getTranslation()), points[0].getTranslation())){
      segments.add(segment);
    }
    
    
  }
  // case for more then 1 segment
  else {
    // creates the first leg
      for(Segment segment : AvoidBannedZone.fixPoint(corners[0].getAtoCurveLeg(), points[0].getTranslation())){
        segments.add(segment);
      }

      // creates arc than leg
      for (int i = 0; i < corners.length - 1; i += 1) {
        

        
        for (Segment segment : AvoidBannedZone.fixPoint(corners[i].getArc(), corners[i].getAtoCurveLeg().getPoints()[0])) {
          segments.add(segment);
        }
        for(Segment segment : AvoidBannedZone.fixPoint(new Leg(corners[i].getCurveEnd(), corners[i + 1].getCurveStart()), points[i].getTranslation())){
          segments.add(segment);
        }
        

      }
      segments.add(corners[corners.length - 1].getArc());
      segments.add(corners[corners.length - 1].getCtoCurveLeg());
      // creates the last arc and leg
    }
  }
   

  
  @Override
  public void initialize() {

    

    distancePassed = 0;
    driveTrapezoid = new Trapezoid(points[0].getVelocity(), PATH_ACCEL, points[1].getVelocity());

    initPoints();
    initCorners();
    initSegments();
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

  public boolean isFinishedSegment(){
    return segments.get(segmentIndex).distancePassed(chassisPose.getTranslation()) >= segments.get(segmentIndex).getLength() - PATH_DISTANCE_OFFSET;
  }
  public boolean isLastSegment(int index){
    return index == segments.size() - 1;
  }
  private boolean isNotFirstSegment(){
    return segmentIndex != 0;
  }

  int angleIndex = 1;
  PIDController pid = new PIDController(0.005, 0, 0);

  @Override
  public void execute() {
    
    chassisPose = chassis.getPose();

    // current velocity vector
    Translation2d currentVelocity = chassis.getVelocity();

    //calc for distance passed based on total left minus distance passed on current segment
    distancePassed = totalLeft - segments.get(segmentIndex).distancePassed(chassisPose.getTranslation());


    
    wantedAngle = points[angleIndex].getRotation();

    //update total left when finished current segment
    if(isFinishedSegment()){
      totalLeft -= segments.get(segmentIndex).getLength();
      if(isNotFirstSegment()) angleIndex++;
      
      
      //update segment index
      if (!isLastSegment(segmentIndex) || segments.get(segmentIndex).getLength() <= 0.15){
            

        driveTrapezoid = new Trapezoid(points[segmentIndex].getVelocity(), accel, points[segmentIndex+1].getVelocity());
        segmentIndex++;
      }
      else driveTrapezoid = new Trapezoid(points[segmentIndex].getVelocity(), accel, finishVel);
    }

    //calc drive velocity
    driveVelocity = driveTrapezoid.calcVelocity(
        totalLeft - segments.get(segmentIndex).distancePassed(chassisPose.getTranslation()),
        currentVelocity.getNorm());

    Translation2d velVector = segments.get(segmentIndex).calc(chassisPose.getTranslation(), driveVelocity);

    
    //case for correct Translation2d but wrong angle so stop chassis but keep rotation
    if (totalLeft <= PATH_DISTANCE_OFFSET) velVector = new Translation2d(0, 0);

    //calc rotation velocity based on PID

    double rotationVelocity = (Math.abs(wantedAngle.minus(chassis.getAngle()).getDegrees()) <= PATH_ANGLE_OFFSET)
      ? 0 : PATH_ROTATION_PID.calculate(wantedAngle.minus(chassis.getAngle()).getRadians(), 0);

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
