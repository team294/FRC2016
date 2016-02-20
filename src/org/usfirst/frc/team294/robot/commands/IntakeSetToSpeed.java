package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeSetToSpeed extends Command {
	double speedToSet; 

	public IntakeSetToSpeed(double speed) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.intake);
		speedToSet = speed; 
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if(speedToSet>1||speedToSet<-1){
		Robot.intake.setSpeed(speedToSet);}
		
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if(speedToSet>1||speedToSet<-1){
		return true; }
		return false; 
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.intake.setSpeed(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
