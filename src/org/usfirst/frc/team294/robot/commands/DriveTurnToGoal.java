package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.utilities.ToleranceChecker;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTurnToGoal extends Command {

	// Initial settings when command was invoked
	private double maxTol;
	
	// Other initialization variables
    private double commandSpeed;
    private double targetAngle;
    
    // Steering settings
    private double angleErr, speedControl;
    private double minSpeed = 0.2;
    private double kPangle = 0.03; 
    
    // Check if target has been reached
    ToleranceChecker angleTol = new ToleranceChecker(1.5, 5);		// Default tolerance of 1.5 will be overwritten in initialize()
    
    /** 
     * Turns robot to face largest goal found, to within tolerance
     * @param angleTolerance = accuracy of robot turning, in degrees
     */
    public DriveTurnToGoal(double angleTolerance) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);        
        requires(Robot.driveTrain);
        
        maxTol = angleTolerance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	angleTol.reset();
    	angleTol.setTolerance(maxTol);
       	Robot.driveTrain.resetDegrees();    		

        commandSpeed = 0.65;
        Robot.vision.findGoal();
        targetAngle = Robot.vision.getGoalXAngleError();
    	System.out.println("Auto turn to goal:  dist = " + Robot.vision.getGoalDistance() + ", turn angle = " + targetAngle);

        targetAngle = (targetAngle < 0) ? targetAngle+360.0 : targetAngle;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	// Find angle error.  - = left, + = right
    	angleErr = targetAngle - Robot.driveTrain.getDegrees();
    	angleErr = (angleErr>180) ? angleErr-360 : angleErr;
    	angleErr = (angleErr<-180) ? angleErr+360 : angleErr;
    	
        if (Robot.smartDashboardDebug) {
        	SmartDashboard.putNumber("Drive Angle Error", angleErr);
        }
        
    	angleTol.check(Math.abs(angleErr));
    	
    	if (angleTol.success()) {
        	Robot.driveTrain.stop();
    	} else {
        	// Find speed to drive
        	speedControl = angleErr*kPangle;
        	speedControl = (speedControl>1) ? 1 : speedControl;
        	speedControl = (speedControl<-1) ? -1 : speedControl;
        	speedControl *= commandSpeed;
        	
        	if (speedControl>0) {
        		speedControl = (speedControl<minSpeed) ? minSpeed : speedControl;
        	} else {
        		speedControl = (speedControl>-minSpeed) ? -minSpeed : speedControl;
        	}
        	
        	Robot.driveTrain.driveCurve(speedControl, 1);
        	SmartDashboard.putNumber("Drive angle power", speedControl);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return angleTol.success();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.driveTrain.stop();
    }
}
