package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ShootBall extends Command {
	Command shootOnly, shootCruise, shootLow;
	
	/**
	 * "Smart" shoot ball, depending on arm angle
	 */
    public ShootBall() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.shooter);
    	requires(Robot.shooterArm);
    	requires(Robot.intake);
    	
    	shootOnly = new ShootBallOnly();
    	shootCruise = new ShootBallMoveArmLow();
    	shootLow = new ShootBallLow();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if (Robot.shooterArm.getAngle()< (RobotMap.shooterArmBallCruiseAngle-3)) {
    		shootLow.start();
    	} else if (Robot.shooterArm.getAngle()< (RobotMap.shooterArmBallCruiseAngle+5)) {
    		shootCruise.start();
    	} else {
    		shootOnly.start();
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
