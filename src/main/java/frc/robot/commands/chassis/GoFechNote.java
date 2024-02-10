// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;

public class GoFechNote extends Command {
  /** Creates a new GoFechNote. */
  Chassis chassis;
  double[] llpython;
  Intake intake;
  private double Dist;
  private double Angle;
  private double Note_X;
  private double Note_Y;
  double rotationVelocity;
  PIDController rotationPidController = new PIDController(0.31, 0.006, 0.0000025);
  public GoFechNote(Chassis chassis, Intake intake) {
    this.chassis = chassis;
    this.intake = intake;
    addRequirements(chassis, intake);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    llpython = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython").getDoubleArray(new double[8]);
    Angle = llpython[1];
    Note_X = llpython[2];
    Note_Y = llpython[3];
    rotationVelocity = (Angle <= 1 && Angle >= -1) ? 0 : rotationPidController.calculate(chassis.getAngle().getDegrees(),Angle)*Math.PI/2;
    ChassisSpeeds speeds = new ChassisSpeeds(1.5,0, rotationVelocity);
    chassis.setVelocities(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Dist == 0;
  }
}
