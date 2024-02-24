// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter.SHOOTER_MOTOR;
import frc.robot.utils.Utils;

public class ActivateShooter extends Command {

  Translation2d from;
  boolean isFollowing;
  boolean isContinious;
  boolean isShooting;
  Timer timer;
  Shooter shooter;
  Intake intake;
  Chassis chassis;
  Translation2d speaker;
  boolean finish = false;

  /** Creates a new ActivateShooter. */
  public ActivateShooter(Shooter shooter, Intake intake, Chassis chassis, boolean isContinious) {
    this(shooter, intake, chassis, null, isContinious);
  }

  public ActivateShooter(Shooter shooter, Intake intake, Chassis chassis, Translation2d from, boolean isContinious) {
    this.isFollowing = from == null;
    this.from = from;
    this.shooter = shooter;
    this.intake = intake;
    this.chassis = chassis;
    this.timer = new Timer();
    this.isContinious = isContinious;
    this.from = from;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speaker = Utils.speakerPosition();
    finish = false;
    timer.reset();
    shooter.isActive(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vUp;
    double Vdown;
    double a;
    if (shooter.isShootingAmp()) {
      vUp = ShooterConstants.AmpPera.UP;
      Vdown = ShooterConstants.AmpPera.DOWN;
      a = ShooterConstants.AmpPera.ANGLE;
    } else {
      Translation2d pos = isFollowing ? chassis.getPose().getTranslation() : from;
      Translation2d vec = speaker.minus(pos);
      double distance = vec.getNorm();
      var angleToVel = Utils.getShootingAngleVelocity(distance);
      System.out.println("dis= "+ distance + "\nv= "+angleToVel.getSecond()+"\na ="+angleToVel.getFirst());
      vUp = angleToVel.getSecond();
      Vdown = vUp;
      a = angleToVel.getFirst();
    }
      double dis = shooter.getDistanceFromAngle(a);
      double disError = dis - shooter.getDis();
      disError = MathUtil.applyDeadband(disError, 2);
      shooter.angleSetPow(-1*MathUtil.clamp(0.05 * Math.signum(disError) + disError * 0.05, -0.4, 0.4));
      shooter.setVel(vUp, Vdown);
      double vError = shooter.getMotorVel(SHOOTER_MOTOR.UP) - vUp;
      boolean isReady = Math.abs(disError) < 1 && Math.abs(vError) < 0.3;
      shooter.isShootingReady(isReady);
      if (!isShooting && shooter.isShooting() && isReady) {
        isShooting = true;
        timer.reset();
        timer.start();
        shooter.feedingSetPow(1);
        intake.setPower(1);
      } else if (isShooting && timer.get() > 0.4) {
        isShooting = false;
        shooter.isShooting(false);
        shooter.isShootingAmp(false);
        shooter.feedingSetPow(0);
        intake.setPower(0);
        finish = !isContinious;
        shooter.isShootingReady(false);
      }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(finish) {
      shooter.stop();
      shooter.feedingStop();
      shooter.angleStop();
      intake.stop();
      shooter.isActive(false);
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish || (isShooting && timer.get() > 0.5);
  }
}
