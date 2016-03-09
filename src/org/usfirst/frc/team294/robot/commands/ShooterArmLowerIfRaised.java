package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ShooterArmLowerIfRaised extends Command {
	boolean shooterArmLowered=false;
	double minimumTime=2.0;

    public ShooterArmLowerIfRaised() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.shooterArm);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(Robot.shooterArm.getAngle()>RobotMap.shooterArmBallLoadAngle&&!Robot.intake.intakeIsUp()){
			Robot.shooterArm.moveToAngle(RobotMap.shooterArmBallLoadAngle);;
			shooterArmLowered=true;
		} else{
			shooterArmLowered=false;
		}

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
		if(shooterArmLowered==true){
			return true;
		}
		if(timeSinceInitialized() >= minimumTime){
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
