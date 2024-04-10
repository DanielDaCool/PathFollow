// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PathFollow.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PathFollow.CommandInPos;

/**
 * 
 *A point in the path
 *
 */
public class pathPoint extends Pose2d{
    double radius;
    double velocity;
    CommandInPos command;

    public pathPoint(Translation2d p, Rotation2d r, double velocity) {
      this(p.getX(),p.getY(),r,0, velocity);
    }
    public pathPoint(Translation2d p, Rotation2d r, double radius, double velocity) {
      this(p.getX(),p.getY(),r,radius, velocity);
    }

    public pathPoint(double x, double y, Rotation2d rotation, double radius, double velocity) {
        super(x,y,rotation);
        this.radius = radius;
        this.velocity = velocity;
      
    }


    public pathPoint(Translation2d t, Rotation2d rotation, double radius, double velocity, CommandInPos commandInPos) {
      super(t.getX(), t.getY(),rotation);
      this.radius = radius;
      this.velocity = velocity;
      this.command = commandInPos;
    }
        public pathPoint(double x, double y, Rotation2d rotation, double radius, double velocity, CommandInPos commandInPos) {
        super(x, y, rotation);
        this.radius = radius;
        this.velocity = velocity;
        this.command = commandInPos;
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
      return command.getCommand();
    }
    public CommandInPos getCommandInPos(){
      return command;
    }
    public void setCommand(CommandInPos command){
      this.command = command;

    }



}
