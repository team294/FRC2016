package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ShootFromLocation;
import org.usfirst.frc.team294.robot.utilities.ToleranceChecker;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Turn shooter flywheels on or off.
 */
public class FlyWheelSetToSpeed extends Command {
	
	protected double topSpeed, bottomSpeed;
	ToleranceChecker sTol = new ToleranceChecker(150, 10);

	/**
	 * Set target speed for shooter flywheels.  Command finishes when flywheel is at speed.
	 * @param speed target speed in RPM.  + kick ball out, - intake ball
	 */
	public FlyWheelSetToSpeed(double speed) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.shooter);
		this.topSpeed = speed;
		this.bottomSpeed = speed;
    }

	/**
	 * Set target speed for shooter flywheels.  Command finishes when flywheels are at speed.
	 * @param topSpeed target speed in RPM.  + kick ball out, - intake ball
	 * @param bottomSpeed target speed in RPM.  + kick ball out, - intake ball
	 */
	public FlyWheelSetToSpeed(double topSpeed, double bottomSpeed) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.shooter);
		this.topSpeed = topSpeed;
		this.bottomSpeed = bottomSpeed;
    }

    public FlyWheelSetToSpeed(ShootFromLocation location) {
        requires(Robot.shooter);
        topSpeed = RobotMap.getTopSpeed(location);
        bottomSpeed = RobotMap.getBottomSpeed(location);
	}
    
	// Called just before this Command runs the first time
    protected void initialize() {
    	Robot.shooter.setSpeed(topSpeed, bottomSpeed);
    	sTol.reset();
    	
    	Robot.shooter.setLEDsFlywheelAtSpeed(false);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	double err;

    	err = Math.abs(topSpeed - Robot.shooter.getTopFlyWheelSpeed()) + Math.abs(bottomSpeed - Robot.shooter.getBottomFlyWheelSpeed()) ; 
    	if (sTol.success(err)) {
        	Robot.shooter.setLEDsFlywheelAtSpeed((topSpeed>0));  // Turn on light if we reached speed (outward only)
    		return true;
    	} else {
    		return false;
    	}
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.shooter.setLEDsFlywheelAtSpeed((topSpeed>0));  // Turn on light if we reached speed (outward only)
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
