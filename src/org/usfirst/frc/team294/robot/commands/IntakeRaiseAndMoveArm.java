package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeRaiseAndMoveArm extends Command {

	boolean intakeExecuted=false;
	public IntakeRaiseAndMoveArm() {
		requires(Robot.intake); 
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if(Robot.shooterArm.getAngle()>RobotMap.upperBoundAngleToAvoid&&!Robot.intake.intakeIsUp()){
			Robot.intake.raiseIntake();
			intakeExecuted=true;
		} else{
			Robot.shooterArm.moveToAngle(RobotMap.upperBoundAngleToAvoid+3);
			Robot.intake.raiseIntake();
			intakeExecuted=true;
		}
		intakeExecuted=false;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.intake.raiseIntake();
	}

	// Make this return true when this Command no longer needs to run execute()
	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if(intakeExecuted==false){
			return true;
		}
		if(timeSinceInitialized() >= 1){
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
