package frc.robot.commands.chassis.Auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.DriveAndPickNote;
import frc.robot.commands.chassis.DriveToNote;
import frc.robot.commands.chassis.GoToAngleChassis;
import frc.robot.commands.chassis.Paths.PathFollow;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeToShooter;
import frc.robot.commands.shooter.ActivateShooter;
import frc.robot.commands.shooter.AngleGoToAngle;
import frc.robot.commands.shooter.ShooterPowering;
import frc.robot.commands.shooter.ShooterShoot;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.CommandParams;
import frc.robot.subsystems.shooter.utils.shootFromAnyPlace;
import frc.robot.utils.Utils;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class StartTOP extends SequentialCommandGroup {
  //shootFromAnyPlace shootFromAnyPlace = new shootFromAnyPlace();
  double wantedAngleClose = 53;
  double wantedVelClose = 15.5;
  double maxVel = 4;
  double maxAceel = 12;
  
  pathPoint[] leaveSpeaker = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false), //doesnt matter because it gets fixed in the command
    new pathPoint(1.4, 6.9, Rotation2d.fromDegrees(0), 0, false)};

  pathPoint[] goToNote1 = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false), //doesnt matter because it gets fixed in the command
    new pathPoint(6.8, 7.47, Rotation2d.fromDegrees(0), 0, false)};
    
    
  pathPoint[] goBackToShoot = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false), //doesnt matter because it gets fixed in the command
    new pathPoint(4.5, 6.58, Rotation2d.fromDegrees(0), 0, false)};

  pathPoint[] goToNote2 = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false), //doesnt matter because it gets fixed in the command
    new pathPoint(6, 6, Rotation2d.fromDegrees(0), 0, false)};
  
  

  
  

  /** Creates a new StartTOP auto. */
  public StartTOP(Chassis chassis, Shooter shooter, Intake intake, boolean isRed) {


  Translation2d speaker = (isRed) ? new Translation2d(16.54 - (-0.04), 5.5) : new Translation2d(-0.04, 5.55);


    addCommands((new AngleGoToAngle(shooter, wantedAngleClose).alongWith(new ShooterPowering(shooter, wantedVelClose)))
    .andThen(new IntakeToShooter(intake, shooter, wantedVelClose).raceWith(new WaitCommand(0.5)))
    .andThen((new DriveToNote(chassis).raceWith(new IntakeCommand(intake)))
    .alongWith((new AngleGoToAngle(shooter, 38).alongWith(new ShooterPowering(shooter, 16.5))))
    .andThen(new GoToAngleChassis(chassis, speaker))
    .andThen(new IntakeToShooter(intake, shooter, 16.5).raceWith(new WaitCommand(0.5)))
    .andThen((new PathFollow(chassis, goToNote1, maxVel, maxAceel, 2, isRed)).raceWith(new WaitUntilCommand(()->Utils.seeNote())))
    .andThen((new DriveToNote(chassis).raceWith(new IntakeCommand(intake))))
    .andThen(new PathFollow(chassis, goBackToShoot, maxVel, maxAceel, 0, isRed)
    .alongWith((new AngleGoToAngle(shooter, 33).alongWith(new ShooterPowering(shooter, 18)))))
    .andThen(new GoToAngleChassis(chassis, speaker))
    .andThen(new IntakeToShooter(intake, shooter, 18).raceWith(new WaitCommand(0.5))))
    .andThen(new PathFollow(chassis, goToNote2, maxVel, maxAceel, 2, isRed)).andThen((new DriveToNote(chassis).raceWith(new IntakeCommand(intake)))
    .andThen(new PathFollow(chassis, goBackToShoot, maxVel, maxAceel, 0, isRed)
    .alongWith((new AngleGoToAngle(shooter, 32.3).alongWith(new ShooterPowering(shooter, 18)))))
    .andThen(new GoToAngleChassis(chassis, speaker))
    .andThen(new IntakeToShooter(intake, shooter, 18).raceWith(new WaitCommand(0.5)))));
     
  }

  private Command takeNote(Chassis chassis, Intake intake) {
    return new DriveToNote(chassis).raceWith(new IntakeCommand(intake));
  }

}

