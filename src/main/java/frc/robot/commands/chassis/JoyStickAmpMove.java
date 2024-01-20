// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Amp;

public class JoyStickAmpMove extends Command {
  private final XboxController xboxController;
  private Amp amp;
  private Translation2d translation2d;
  /** Creates a new JoyStickAmpMove. */
  public JoyStickAmpMove(XboxController xboxController, Amp amp) {
    this.xboxController = xboxController;
    this.amp = amp;
    addRequirements(amp);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    translation2d = Amp.getStickRight(xboxController);
    amp.setPowers(translation2d.getY(), 0);
  }

  @Override
    public void end(boolean interrupted) {
        amp.stop();
    }
}