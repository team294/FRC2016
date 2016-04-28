package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ShooterArmMoveToDashboardAngle extends Command {
	private double targetArmAngle;

	/**
	 * Moves arm to angle of goal based on camera and holds there with PID/potentiometer.
	 */
    public ShooterArmMoveToDashboardAngle() {
    	requires(Robot.shooterArm); 
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	targetArmAngle = SmartDashboard.getNumber("Shooter Arm 'Dashboard angle'", 0.0);;
    	Robot.shooterArm.moveToAngle(targetArmAngle);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return Robot.shooterArm.moveToAngleIsFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
