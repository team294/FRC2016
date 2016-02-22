package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeLowerIfRaised extends Command {

	boolean intakeExecuted=false;
	public IntakeLowerIfRaised() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.intake);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if(Robot.shooterArm.getAngle()<RobotMap.lowerBoundAngleToAvoid&&Robot.intake.intakeIsUp()){
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
