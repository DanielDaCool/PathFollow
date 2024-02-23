
package frc.robot.commands.chassis.Paths;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.shooter.ActivateShooter;
import frc.robot.commands.shooter.AngleGoToAngle;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.AmpPera;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class GoToAMP extends SequentialCommandGroup {
  
  
  public GoToAMP(Shooter shooter, Chassis chassis, Intake intake, boolean isRed) {

    pathPoint[] pointsToAmp = {
      new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false),
      new pathPoint(14.321, 0.1, Rotation2d.fromDegrees(-90), 0.25, false)
    };
    addCommands(new PathFollow(chassis, pointsToAmp, 4, 12, 1, isRed).alongWith(new AngleGoToAngle(shooter, AmpPera.ANGLE))
    .andThen(new ActivateShooter(shooter, intake, chassis, chassis.getPose().getTranslation(), true, true)));
  }
}
