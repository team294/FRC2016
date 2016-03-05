package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Moves the shooter arm on the shortest path that is out of the way of the intake. Takes no parameters. 
 * If the path lengths are equal, moves the shooter arm down. If the arm doesn't interfere with the intake
 * does nothing.
 */
public class ShooterArmMoveAwayFromIntake extends Command {
	private boolean needToMoveArm;

	/**
	 * Moves the shooter arm on the shortest path that is out of the way of the intake. Takes no parameters. 
	 * If the path lengths are equal, moves the shooter arm down. If the arm doesn't interfere with the intake
	 * does nothing.
	 */
    public ShooterArmMoveAwayFromIntake() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.shooterArm);
    	requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	needToMoveArm = Robot.intake.shooterArmConflicts();
		if(needToMoveArm) {
	    	Robot.intake.setSpeed(0);

	    	if(Math.abs(Robot.shooterArm.getAngle()-RobotMap.upperBoundAngleToAvoid) < 	
					Math.abs(Robot.shooterArm.getAngle()-RobotMap.lowerBoundAngleToAvoid)) {
				Robot.shooterArm.moveToAngle(RobotMap.upperBoundAngleToAvoid+3);
			} else {
				Robot.shooterArm.moveToAngle(RobotMap.lowerBoundAngleToAvoid-3);
			}
		} 
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if (needToMoveArm) {
    		return (Robot.shooterArm.moveToAngleIsFinished());
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
