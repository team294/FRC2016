package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ShooterArmMoveToGoal extends Command {
	private double targetArmAngle;

	/**
	 * Moves arm to angle of goal based on camera and holds there with PID/potentiometer.
	 */
    public ShooterArmMoveToGoal() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);

    	requires(Robot.shooterArm); 
    }

    // Called just before this Command runs the first time
    protected void initialize() {
       	Robot.vision.findGoal();
    	targetArmAngle = Robot.vision.getGoalArmAngle();
    	Robot.shooterArm.moveToAngle(targetArmAngle);
    	Robot.writeLog("Auto move arm to goal (init):  dist = " + Robot.vision.getGoalDistance() + ", target arm angle = " + targetArmAngle);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if (Robot.shooterArm.moveToAngleIsFinished()){
        	Robot.writeLog("Auto move arm to goal (finish):  target arm angle = " + targetArmAngle + ", actual arm angle = " + 
        			Robot.shooterArm.getAngle());
    		return true;
    	} else {
    		return false;
    	}
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
