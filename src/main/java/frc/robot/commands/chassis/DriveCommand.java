package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.utils.LedControll;

import static frc.robot.subsystems.chassis.ChassisConstants.*;

public class DriveCommand extends Command {
  private final Chassis chassis;
  private final PS4Controller controller;
  private final CommandXboxController commandXboxController;
  private LedControll led;

  private double direction;

  private boolean precisionDrive = false;
  Rotation2d wantedAngleApriltag = new Rotation2d();
  boolean rotateToApriltag = false;
  PIDController rotationPidController = new PIDController(0.03, 0,0.0008);


  public DriveCommand(Chassis chassis, PS4Controller controller, CommandXboxController commandXboxController, boolean isRed) {
    this.chassis = chassis;
    this.commandXboxController = commandXboxController;
    this.controller = controller;
    direction = isRed ? 1 : -1;
    led = new LedControll(9, 100);
    addRequirements(chassis);
    // commandXboxController.b().onTrue(new InstantCommand(() -> precisionDrive = !precisionDrive));
    // commandXboxController.y().onTrue(new InstantCommand((() -> this.wantedAngleApriltag = chassis.getClosetAngleApriltag())).andThen(() -> rotateToApriltag = true));
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double joyX = deadband(commandXboxController.getLeftY(), 0.1) * direction;
    double joyY = deadband(commandXboxController.getLeftX(), 0.1) * direction;

    

    double rot = -(deadband(commandXboxController.getLeftTriggerAxis(), 0.1) - deadband(commandXboxController.getRightTriggerAxis(), 0.1));
    
    if(rot != 0){
      rotateToApriltag = false;
    }

    double velX = Math.pow(joyX, 2)* MAX_DRIVE_VELOCITY * Math.signum(joyX);
    double velY = Math.pow(joyY, 2) * MAX_DRIVE_VELOCITY * Math.signum(joyY);
    double velRot = Math.pow(rot, 2) * MAX_OMEGA_VELOCITY * Math.signum(rot);

    if(rotateToApriltag)
    {
      
      if(Math.abs(
        (wantedAngleApriltag).minus(chassis.getAngle()).getDegrees()) >= -1 &&
      Math.abs((wantedAngleApriltag).minus(chassis.getAngle()).getDegrees()) <= 1) {
        velRot = 0;
        rotateToApriltag = false;
      }
      else{
        velRot = rotationPidController.calculate(chassis.getAngle().getDegrees(),wantedAngleApriltag.getDegrees())*Math.toRadians(180);
      }
     
    }
    if (precisionDrive) {
      velX /= 4;
      velY /= 4;
      velRot /= 4;
    }

    //System.out.println("target velocity= " + velRot);

    llpython = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython").getDoubleArray(new double[8]);
    Dist = llpython[0];
    if(Dist != 0){
      if(Dist <= 150){
        led.blink(0, 255, 0);
      }
      else{
        led.setColor(0, 255, 0);
      }
    }



    ChassisSpeeds speeds = new ChassisSpeeds(velX, velY, velRot);
    chassis.setVelocities(speeds);
  }

  private double deadband(double x, double threshold) {
    return (Math.abs(x) < threshold)?0:x;
  }
}
