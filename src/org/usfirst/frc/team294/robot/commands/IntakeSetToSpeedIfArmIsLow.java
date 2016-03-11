package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Set intake to given speed, but only if arm is below 20 degrees.
 * This is intended for a "smart shooting sequence" that can shoot
 * high or low.  Waits 0.2 sec to wait for initial current spike,
 * so that we don't start the shooter motors simultaneously. 
 */
public class IntakeSetToSpeedIfArmIsLow extends Command {
	double speedToSet; 
	boolean speedWasSet;
	
	/**
	 * Set intake to given speed, but only if arm is below 20 degrees.
	 * This is intended for a "smart shooting sequence" that can shoot
	 * high or low.  Waits 0.2 sec to wait for initial current spike,
	 * so that we don't start the shooter motors simultaneously. 
	 * @param speed +1 = full in, -1 = full out, 0 = off
	 */
	public IntakeSetToSpeedIfArmIsLow(double speed) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.intake);
		speedToSet = speed; 
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		speedWasSet = (speedToSet<=1 && speedToSet>=-1 && Robot.shooterArm.getAngle() <= 20); 
		
		if (speedWasSet) {
			Robot.intake.setSpeed(speedToSet);
		}		
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (speedWasSet) {
			return (timeSinceInitialized() >= 0.2);
		} else {
			return true;
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
