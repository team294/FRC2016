package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Drive the motors at "speed" and "curve". 
 * Both outputMagnitude and curve are -1.0 to +1.0 values, where 0.0 represents stopped and not turning. 
 * curve < 0 will turn left and curve > 0 will turn right. The algorithm for steering provides a constant
 * turn radius for any normal speed range, both forward and backward. Increasing m_sensitivity causes 
 * sharper turns for fixed values of curve. This function will most likely be used in an autonomous 
 * routine.
 */
public class DriveCurve extends Command {
	double speed, curve;	
	
	/**
	 * Drive the motors at "speed" and "curve". 
	 * Both outputMagnitude and curve are -1.0 to +1.0 values, where 0.0 represents stopped and not turning. 
	 * curve < 0 will turn left and curve > 0 will turn right. The algorithm for steering provides a constant
	 * turn radius for any normal speed range, both forward and backward. Increasing m_sensitivity causes 
	 * sharper turns for fixed values of curve. This function will most likely be used in an autonomous 
	 * routine.
	 * @param speed The speed setting for the outside wheel in a turn, forward or backwards, +1 to -1.
	 * @param curve The rate of turn, constant for different forward speeds. Set curve < 0 for left turn or 
	 *          curve > 0 for right turn. Set curve = e^(-r/w) to get a turn radius r for wheelbase w of 
	 *          your robot. Conversely, turn radius r = -ln(curve)*w for a given value of curve and wheelbase w
	 */
    public DriveCurve(double speed, double curve) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    	
    	this.speed = speed;
    	this.curve = curve;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveTrain.driveCurve(speed, curve);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.driveTrain.stop();
    }
}
