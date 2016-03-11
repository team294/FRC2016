package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Determines if (1) intake is up and (2) arm is not in the way.
 * If so, then it activates the solenoid to lower the intake and waits 2 seconds for intake to lower.
 */
public class IntakeLowerIfRaised extends Command {

	boolean intakeExecuted=false;
	
	/**
	 * Determines if (1) intake is up and (2) arm is not in the way.
	 * If so, then it activates the solenoid to lower the intake and waits 2 seconds for intake to lower.
	 */
	public IntakeLowerIfRaised() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.intake);
		requires(Robot.shooterArm);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if(!Robot.intake.shooterArmConflicts() && (Robot.intake.intakeIsUp() || Robot.intake.intakeSolenoidIsOff() ) ){
			Robot.intake.lowerIntake();
			intakeExecuted=true;
		} else{
			intakeExecuted=false;
		}

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if(intakeExecuted==false){
			return true;
		}
		if(timeSinceInitialized() >= 2){
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
