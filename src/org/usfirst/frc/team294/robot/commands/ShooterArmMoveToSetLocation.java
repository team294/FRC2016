package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ShooterArmMoveToSetLocation extends Command {
double angleToMove ; 
    public ShooterArmMoveToSetLocation(double angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	if(angle == RobotMap.shootingAngle){
    		switch(Robot.oi.readTopKnob()){
    		case minus1degree:
    			angle -= 1;
    			break;
    		case minus2degrees:
    			angle -= 2;
    			break;
    		case minus3degrees:
    			angle -= 3;
    			break;
    		case minus4degrees:
    			angle -= 4;
    			break;
    		case minus5degrees:
    			angle -= 5;
    			break;
    		case minus6degrees:
    			angle -= 6;
    			break;
    		case noChange:
    			break;
    		case plus1degree:
    			angle += 1;
    			break;
    		case plus2degrees:
    			angle += 2;
    			break;
    		case plus3degrees:
    			angle += 3;
    			break;
    		case plus4degrees:
    			angle += 4;
    			break;
    		case plus5degrees:
    			angle += 5;
    			break;
    		default:
    			break;
    		}
    	}
    	
    	this.angleToMove = angle; 
    	requires(Robot.shooterArm); 
    	requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.intake.setSpeed(0);
    	Robot.shooterArm.moveToAngle(angleToMove);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return (Robot.shooterArm.moveToAngleIsFinished());
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
