package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class WaitForArmNearSetpoint extends Command {
	double tol;
	
	/**
	 * Waits until the arm is within a number of degrees of the setpoint
	 * @param tolDegrees, tolerance to setpoint (+ or -), in degrees
	 */
    public WaitForArmNearSetpoint(double tolDegrees) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	tol = tolDegrees;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(Robot.shooterArm.getSetpointAngle() - Robot.shooterArm.getAngle()) <= tol;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
