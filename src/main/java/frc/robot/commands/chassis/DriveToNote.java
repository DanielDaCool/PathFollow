
package frc.robot.commands.chassis;

import static frc.robot.subsystems.chassis.ChassisConstants.MAX_OMEGA_ACCELERATION;
import static frc.robot.subsystems.chassis.ChassisConstants.MAX_OMEGA_VELOCITY;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;
import edu.wpi.first.wpilibj.Timer;

public class DriveToNote extends Command {
  Chassis chassis;
  double velocity;
  double lastDistance;

  double[] llpython;
  double distance;
  double angle;
  double lastAngle;
  NetworkTableEntry llentry;
  ChassisSpeeds speed;
  boolean finish = false;
  boolean countTime;

  PIDController rotationPidController = new PIDController(0.04, 0.00, 0.006);

  Timer timer = new Timer();

  public static boolean isStart = false;

  public DriveToNote(Chassis chassis, double vel, boolean countTime) {
    this.chassis = chassis;
    this.velocity = vel;
    this.countTime = countTime;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    isStart = true;
    lastDistance = 0;
    distance = 0;
    llentry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython");
    llpython = llentry.getDoubleArray(new double[8]);
    finish = llpython[0] == 0;
    timer.start();
  }

  private double calcTimeToRotate(double angle){
    return 2 * (Math.sqrt(Math.abs(angle) / MAX_OMEGA_ACCELERATION));
  }

  @Override
  public void execute() {
    if (finish) {
      return;
    }
    llpython = llentry.getDoubleArray(new double[8]);
    distance = llpython[0];
    angle = llpython[1];
    if (distance > 0) {
      timer.reset();
    } else {
      return;
    }
   //double rotateVel = (Math.abs(angle - 3) <= 3) ? 0 : rotationPidController.calculate(-angle, 3);
    double fieldRelativeAngle = angle + chassis.getAngle().getDegrees();
    fieldRelativeAngle = Math.toRadians(fieldRelativeAngle); 
    velocity = Math.min((distance - 7) / calcTimeToRotate(angle), 4);
    speed = new ChassisSpeeds(velocity * Math.cos(fieldRelativeAngle), velocity * Math.sin(fieldRelativeAngle), MAX_OMEGA_VELOCITY);

    lastDistance = distance;
    lastAngle = fieldRelativeAngle;
    chassis.setVelocitiesRotateToAngle(speed, Rotation2d.fromDegrees(fieldRelativeAngle));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
    isStart = false;
  }

  @Override
  public boolean isFinished() {
    return finish || (timer.get() > 1.5 && countTime);
  }

}

