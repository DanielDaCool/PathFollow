package frc.robot.PathFollow;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class CommandInPos extends Translation2d {
    public Command wantedCommand;
    private double distanceMaxOffset = 0.4;
    private boolean firstTime = true;
    public CommandInPos(Translation2d point, Command wantedCommand){
        super(point.getX(), point.getY());
        this.wantedCommand = wantedCommand;
    }
    public Command getCommand(){
        return wantedCommand;
    }
    public void setCommand(Command command){
        this.wantedCommand = command;
    }
    private void scheduleCommand(){
        wantedCommand.schedule();
    }
    public void scheduleIfInPos(Translation2d currentPos){
        if(getDistance(currentPos) <= distanceMaxOffset && firstTime){
            
            scheduleCommand();
            firstTime = false;
        }
        
    }
    
}
