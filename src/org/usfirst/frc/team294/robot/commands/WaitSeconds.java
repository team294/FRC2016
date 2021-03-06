package org.usfirst.frc.team294.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Wait specified number of seconds.  This is useful for command sequences.
 */
public class WaitSeconds extends Command {
	double duration;
	
	/**
	 * Wait specified number of seconds.  This is useful for command sequences.
	 * @param duration = number of seconds to wait
	 */
    public WaitSeconds(double duration) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.duration = duration;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(timeSinceInitialized() >= duration){
    		return true;
    	}
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
