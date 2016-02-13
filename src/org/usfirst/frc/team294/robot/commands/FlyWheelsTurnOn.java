package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class FlyWheelsTurnOn extends Command {
	
	boolean start;
	//double err1 = 0, err2 = 0, err3 = 0, err4 = 0;

    public FlyWheelsTurnOn(boolean start) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.shooter);
    	this.start = start;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
//    	if(!Robot.shooter.isButtonPressed() && start){
//    		return;
//    	}
    	if(start){
    		Robot.shooter.setSpeed(4000);
    	}
    	if(!start){
//    		SmartDashboard.putString("Stop", "It should have stopped.");
    		Robot.shooter.setSpeed(0);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	SmartDashboard.putNumber("Motor_Error", Robot.shooter.getTopError());
//    	if(!Robot.shooter.isButtonPressed()){
//    		return true;
//    	}
//    	if(System.currentTimeMillis()-this.startTime > 500){
//    		err4 = err3;
//    		err3 = err2;
//    		err2 = err1;
//    		err1 = 
//    		if(err1+err2+err3+err4 < 20)
//    			return true;
//    	}
    	//This is the timing out of the command, if the other one doesnt fire first, this one will be there to catch it
    	SmartDashboard.putNumber("Timer", this.timeSinceInitialized());
    	if(this.timeSinceInitialized() > 2){
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
