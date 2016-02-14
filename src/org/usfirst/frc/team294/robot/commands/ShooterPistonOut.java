package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ShooterPistonOut extends Command {
	
	boolean status;

	/**
	 * Move the shooter piston in or out.
	 * @param status = true to move piston out, false to move piston in
	 */
    public ShooterPistonOut(boolean status) {
    	requires(Robot.shooter);
    	//Status = true means the piston is OUT
    	//Status = false means piston is IN
    	this.status = status;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(status)
    		Robot.shooter.setShooterPistonOut();
    	else if(!status)
    		Robot.shooter.setShooterPistonIn();
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
