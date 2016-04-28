package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class VisonSaveFoundGoalState extends Command {

	/**
	 * Looks for goal and saves the found goal state.
	 */
    public VisonSaveFoundGoalState() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.vision);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	boolean bFound;
    	
    	bFound = Robot.vision.findGoal();
    	Robot.writeLog("VisionSaveFoundGoalState:  goal found = " +  bFound);
    	Robot.vision.saveGoalFoundState();
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
