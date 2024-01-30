// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.TrapezoidCalc;

import static frc.robot.Constants.ShooterConstants.*;

public class GoToAngle extends Command {
    
    Shooter shooter;
    double wantedAngle;
    TrapezoidCalc calc;
    double wantedDis;
    double maxVel;
    double acc;
    double endVel;
    double startDis;

    /** Creates a new GoToAngle. */
    public GoToAngle(Shooter shooter, double angle, double maxVel, double acc) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.shooter = shooter;
        this.wantedAngle = angle;
        this.calc = new TrapezoidCalc();
        this.maxVel = maxVel;
        this.acc = acc;
        this.endVel = 0;
        addRequirements(shooter);
    }

    /** Creates a new GoToAngle. */
    public GoToAngle(Shooter shooter, double angle, double maxVel, double acc, double endVel) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.shooter = shooter;
        this.wantedAngle = angle;
        this.calc = new TrapezoidCalc();
        this.maxVel = maxVel;
        this.acc = acc;
        this.endVel = endVel;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    /**
     * @see <a href="https://www.desmos.com/calculator/4ja9zotx82">Desmos Graph</a>
     */
    @Override
    public void initialize() {
        shooter.angleBrake();

        // wantedDis =  KB * Math.cos(wantedAngle * Math.PI / 180)+Math.sqrt(Math.pow(KA, 2) - Math.pow((KB * Math.sin(wantedAngle * Math.PI / 180)), 2));
        wantedDis = KA * Math.cos(wantedAngle * Math.PI / 180) + Math.sqrt(Math.pow(KA, 2) * Math.pow(Math.cos(wantedAngle * Math.PI / 180), 2) - Math.pow(KA, 2) + Math.pow(KB, 2));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.anlgeSetVel(calc.trapezoid(shooter.getAngleVel(), maxVel, endVel, acc, wantedDis - shooter.getDis()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.anlgeStop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        if (!shooter.limits()){
            if (!((wantedDis - startDis > 0) && (shooter.getDis() >= wantedDis))){
                if (!((wantedDis - startDis < 0) && (shooter.getDis() <= wantedDis))){
                    if (!(Math.abs(wantedDis - shooter.getDis()) < 1)){
                        return false;
                    }
                }
            }
        }

        return true;
    }
}
