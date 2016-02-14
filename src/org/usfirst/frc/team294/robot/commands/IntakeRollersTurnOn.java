package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeRollersTurnOn extends Command {

	boolean status;
	double err1 = 0, err2 = 0, err3 = 0, err4 = 0;
	long startTime;

	public IntakeRollersTurnOn(boolean status) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.intake);
		this.status = status;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
    	startTime = System.currentTimeMillis();
    	err1 = err2 = err3 = err4 = Robot.shooter.getTopError();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (status) {
			Robot.intake.setSpeed(-500);
		}
		if (!status) {
			Robot.intake.setSpeed(0);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
//		if (Robot.intake.isButtonPressed() || !status){
//			return true;
//		}
		if (System.currentTimeMillis() - this.startTime > 2000) {
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
