package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Rotate extends Command {

    public Rotate() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.motor1);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.motor1.resetDegrees();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.motor1.driveLeft(.5);
    	Robot.motor1.driveRight(.5);
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if (Math.abs(Robot.motor1.getDegrees())>90) {return true;}
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
