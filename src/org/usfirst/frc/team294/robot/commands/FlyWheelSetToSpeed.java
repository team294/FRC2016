package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.utilities.ToleranceChecker;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Turn shooter flywheels on or off.
 */
public class FlyWheelSetToSpeed extends Command {
	
	int speed;
	ToleranceChecker sTol = new ToleranceChecker(150, 5);

	/**
	 * Set target speed for shooter flywheels.  Command finishes when flywheel is at speed.
	 * @param speed target speed in RPM.  + kick ball out, - intake ball
	 */
	public FlyWheelSetToSpeed(int speed) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.shooter);
		this.speed = speed;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.shooter.setSpeed(speed);
    	sTol.reset();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	double err;
    	
    	Robot.shooter.updateSmartDashboard();

    	err = Math.abs(speed - Robot.shooter.getTopFlyWheelSpeed()) + Math.abs(speed - Robot.shooter.getBottomFlyWheelSpeed()) ; 
    	return sTol.success(err);
    }

    // Called once after isFinished returns true
    protected void end() {
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
