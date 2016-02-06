package org.usfirst.frc294.RobotBuilderTest.commands;

import org.usfirst.frc294.RobotBuilderTest.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class flyWheels extends Command {
	
	boolean start;
	long startTime;
	double err1 = 0, err2 = 0, err3 = 0, err4 = 0;

    public flyWheels(boolean start) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.shooter);
    	this.start = start;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	startTime = System.currentTimeMillis();
    	err1 = err2 = err3 = err4 = Robot.shooter.getTopError();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//System.out.println(Robot.shooter.getTopError());
    	if(!Robot.shooter.isButtonPressed() && start){
    		return;
    	}
    	if(start){
    		Robot.shooter.setSpeed(5000);
    	}
    	if(!start){
    		Robot.shooter.setSpeed(0);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(!Robot.shooter.isButtonPressed()){
    		System.out.println(Robot.shooter.isButtonPressed());
    		return true;
    	}
    	if(System.currentTimeMillis()-this.startTime > 500){
    		err4 = err3;
    		err3 = err2;
    		err2 = err1;
    		System.out.println(err1 + " " + err2 + " " +err3 + " " + err4);
    		if(err1+err2+err3+err4 < 20)
    			return true;
    	}
    	//This is the timing out of the command, if the other one doesnt fire first, this one will be there to catch it
    	if(System.currentTimeMillis()-this.startTime > 2000){
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
