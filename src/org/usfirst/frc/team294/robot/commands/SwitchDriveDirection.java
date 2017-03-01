package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Change the drive direction of the robot
 */
public class SwitchDriveDirection extends Command {

	private boolean direction;
	
	/**
	 * Change the drive direction of the robot
	 * @param direction true to drive towards shooter, false to drive towards gears
	 */
    public SwitchDriveDirection(boolean direction) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
//    	requires(Robot.driveTrain);
    	this.direction = direction;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.oi.setDriveDirection(direction);
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
