package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmPistonOut extends Command {

	private boolean pistonMoved;
	
    public ArmPistonOut() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.armPiston);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(Robot.shooterArm.getAngle() < RobotMap.maxPistonOutAngle && !Robot.armPiston.pistonIsOut()){
    		Robot.armPiston.pistonOut();
    		pistonMoved = true;
    	}else{
        	pistonMoved = false;
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(this.timeSinceInitialized() > 1 || !pistonMoved){
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
