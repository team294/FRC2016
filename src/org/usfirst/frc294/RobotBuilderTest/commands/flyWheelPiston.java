package org.usfirst.frc294.RobotBuilderTest.commands;

import org.usfirst.frc294.RobotBuilderTest.Robot;
import org.usfirst.frc294.RobotBuilderTest.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class flyWheelPiston extends Command {
	
	Solenoid shooterPiston = RobotMap.shooterPiston;
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
    		shooterPiston.set(true);
    	else if(!status)
    		shooterPiston.set(false);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(shooterPiston.get() == status){
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
