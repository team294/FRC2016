package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveForward extends Command {
double leftInitialDistance;
double rightInitialDistance;
    public DriveForward() {
      requires(Robot.motor1); 
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	 leftInitialDistance = Robot.motor1.getLeftEncoder();
    	 rightInitialDistance = Robot.motor1.getRightEncoder(); 
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.motor1.driveLeft(-.5);
    	Robot.motor1.driveRight(.5);
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    if(Math.abs(Robot.motor1.getLeftEncoder()-leftInitialDistance) >1)return true;
return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
