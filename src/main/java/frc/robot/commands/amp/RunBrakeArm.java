// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amp;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.amp.AmpConstants.Parameters;

public class RunBrakeArm extends Command {
  Amp amp;
  boolean lock;
  double startTime;
  /** Creates a new SnowBlowerRun. */
  public RunBrakeArm(Amp amp, boolean lock) {
    this.amp = amp;

    this.lock = lock;
    addRequirements(amp);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    //amp.setVel(0);
    if(lock && !amp.isLocked) { 
      amp.setPowerSnowblower(Parameters.ARM_BRAKE_POW);
    } else if(!lock && amp.isLocked) {
      amp.setPowerSnowblower(Parameters.ARM_BRAKE_POW);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    amp.setPowerSnowblower(0);
    if(lock) { 
      amp.setPowerArm(0);
      amp.isLocked = true;
    } else {
      amp.isLocked = false;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(lock) {
      System.out.println(" brake current = " + amp.getSnowblowerA());
      return amp.isLocked || amp.getSnowblowerA()>Parameters.ARM_BRAKE_MAX_A;
    } else {
      return !amp.isLocked || Timer.getFPGATimestamp()-startTime >0.7;
    }
  }
}
