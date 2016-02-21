package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ShooterArmMoveToSetLocation extends Command {
double angleToMove ; 
    public ShooterArmMoveToSetLocation(double angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.angleToMove = angle; 
    	requires(Robot.shooterArm); 
    	requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.intake.setSpeed(0);
    	Robot.shooterArm.moveToAngle(angleToMove);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return (Robot.shooterArm.moveToAngleIsFinished());
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
