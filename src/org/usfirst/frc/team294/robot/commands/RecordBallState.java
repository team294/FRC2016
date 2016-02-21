package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RecordBallState extends Command {
	boolean ballIsLoaded;
	
    public RecordBallState(boolean ballIsLoaded) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.ballIsLoaded = ballIsLoaded;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.shooter.recordBallState(ballIsLoaded);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
