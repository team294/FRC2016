package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class flyWheelPiston extends Command {
	
	DoubleSolenoid shooterPiston = RobotMap.shooterPiston;
	boolean status;

    public flyWheelPiston(boolean status) {
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
    		shooterPiston.set(DoubleSolenoid.Value.kForward);
    	else if(!status)
    		shooterPiston.set(DoubleSolenoid.Value.kReverse);
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
