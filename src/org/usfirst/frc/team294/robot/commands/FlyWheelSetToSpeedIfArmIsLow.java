package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.utilities.ToleranceChecker;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Turn shooter flywheels on, but only if arm is cruising angle or lower
 */
public class FlyWheelSetToSpeedIfArmIsLow extends Command {
	boolean speedWasSet;
	double topSpeed, bottomSpeed;
	ToleranceChecker sTol = new ToleranceChecker(150, 10);

	/**
	 * Set target speed for shooter flywheels.  Command finishes when flywheel is at speed.
	 * <p> DOES NOTHING if armi is cruising angle or lower
	 * @param speed target speed in RPM.  + kick ball out, - intake ball
	 */
	public FlyWheelSetToSpeedIfArmIsLow(double speed) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.shooter);
		this.topSpeed = speed;
		this.bottomSpeed = speed;
    }

	/**
	 * Set target speed for shooter flywheels.  Command finishes when flywheels are at speed.
	 * <p> DOES NOTHING if armi is cruising angle or lower
	 * @param topSpeed target speed in RPM.  + kick ball out, - intake ball
	 * @param bottomSpeed target speed in RPM.  + kick ball out, - intake ball
	 */
	public FlyWheelSetToSpeedIfArmIsLow(double topSpeed, double bottomSpeed) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.shooter);
		this.topSpeed = topSpeed;
		this.bottomSpeed = bottomSpeed;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		speedWasSet = (Robot.shooterArm.getAngle() <= 20); 
		
		if (speedWasSet) {
			Robot.shooter.setSpeed(topSpeed, bottomSpeed);
	    	sTol.reset();
	    	
	    	Robot.shooter.setFlywheelSpeedLight(false);
		}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if (!speedWasSet) return true;
    	
    	double err;

    	err = Math.abs(topSpeed - Robot.shooter.getTopFlyWheelSpeed()) + Math.abs(bottomSpeed - Robot.shooter.getBottomFlyWheelSpeed()) ; 
    	return sTol.success(err);
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.shooter.setFlywheelSpeedLight((topSpeed>0)&&(speedWasSet));  // Turn on light if we reached speed (outward only)
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
