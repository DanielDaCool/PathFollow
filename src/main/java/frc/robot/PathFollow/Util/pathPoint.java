// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PathFollow.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PathFollow.PathFollowConstants.TimeOfCommand;

/**
 * 
 *A point in the path
 *
 */
public class pathPoint extends Pose2d{
    double radius;
    double velocity;
    Command command;
    TimeOfCommand timeOfCommand;
    boolean waitUntilCommandIsFinished;

    public pathPoint(Translation2d p, Rotation2d r, double velocity) {
      this(p.getX(),p.getY(),r,0, velocity);
      this.waitUntilCommandIsFinished = false;
    }
    public pathPoint(Translation2d p, Rotation2d r, double radius, double velocity) {
      this(p.getX(),p.getY(),r,radius, velocity);
      this.waitUntilCommandIsFinished = false;
    }

    public pathPoint(double x, double y, Rotation2d rotation, double radius, double velocity) {
        super(x,y,rotation);
        this.radius = radius;
        this.velocity = velocity;
        this.waitUntilCommandIsFinished = false;
      
    }


    public pathPoint(Translation2d t, Rotation2d rotation, double radius, double velocity, Command command, TimeOfCommand timeOfCommand, boolean waitUntilCommandIsFinished) {
      super(t.getX(), t.getY(),rotation);
      this.radius = radius;
      this.velocity = velocity;
      this.command = command;
      this.timeOfCommand = timeOfCommand;
      this.waitUntilCommandIsFinished = waitUntilCommandIsFinished;
    }
    public pathPoint(double x, double y, Rotation2d rotation, double radius, double velocity, Command command, TimeOfCommand timeOfCommand, boolean waitUntilCommandIsFinished) {
      super(x, y, rotation);
      this.radius = radius;
      this.velocity = velocity;
      this.command = command;
      this.timeOfCommand = timeOfCommand;
      this.waitUntilCommandIsFinished = waitUntilCommandIsFinished;
    }
    
    public double getRadius()
    {
      return radius;
    }
    public void setRadius(double radius)
    {
      this.radius = radius;
    }
    public double getVelocity(){
      return velocity;
    }
    public void setVelocity(double velocity){
      this.velocity = velocity;
    }
    public Command getCommand(){
      return command;
    }
    public void setCommand(Command command){
      this.command = command;
    }
    public TimeOfCommand getTimeOfCommand(){
      return timeOfCommand;
    }
    public boolean getWaitStatus(){
      return waitUntilCommandIsFinished;
    }
    
    @Override
    public String toString(){
      return "x: " + this.getX() + " y: " + this.getY() + " radius: " + this.getRadius() + " velocity: " + this.getVelocity();
    }



}
