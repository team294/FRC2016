package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.utilities.ToleranceChecker;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Turns a given angle using the NavX.
 */
public class DriveAngle extends Command {

	// Initial settings when command was invoked
    protected double commandSpeed;
    protected double targetAngle;
    protected boolean relative;
	protected double maxTol;
	
    // Check if target has been reached
    private double defaultTolerance = 4.0;
    protected ToleranceChecker angleTol = new ToleranceChecker(defaultTolerance, 5);
    
    // Steering settings
    private double angleErr, speedControl;
    private double priorAngleErr;
    private double minSpeed = 0.22;   // Was 0.25, oscillates sometimes
    private double kPangle = 0.025; 
    
    /**
     * Turns a given angle using the NavX.
     * @param speed +1 = full speed, 0  = don't move
     * @param angle to turn, in degrees.  0 to 360 degrees clockwise, or 0 to -180 counter-clockwise
     * @param relative true = relative to current angle, false = absolute NavX orientation
     */
    public DriveAngle(double speed, double angle, boolean relative) {
        commandSpeed = Math.abs(speed);
        targetAngle = (angle < 0) ? angle+360.0 : angle;
        this.relative = relative;
        maxTol = defaultTolerance;
        
        requires(Robot.driveTrain);
    }

    /**
     * Turns a given angle using the NavX.
     * @param speed +1 = full speed, 0  = don't move
     * @param angle to turn, in degrees.  0 to 360 degrees clockwise, or 0 to -180 counter-clockwise
     * @param relative true = relative to current angle, false = absolute NavX orientation
     * @param angleTolerance = accuracy of robot turning which is good enough to stop command, in degrees
     */
    public DriveAngle(double speed, double angle, boolean relative, double angleTolerance) {
        commandSpeed = Math.abs(speed);
        targetAngle = (angle < 0) ? angle+360.0 : angle;
        this.relative = relative;
        maxTol = angleTolerance;

        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	angleTol.reset();
    	angleTol.setTolerance(maxTol);
    	if (relative) {
        	Robot.driveTrain.resetDegrees();    		
    	}

    	priorAngleErr = getAngleError();
    }

    private double getAngleError() {
    	double angleErr;
    	
    	// Find angle error.  - = left, + = right
    	angleErr = targetAngle - Robot.driveTrain.getDegrees();
    	angleErr = (angleErr>180) ? angleErr-360 : angleErr;
    	angleErr = (angleErr<-180) ? angleErr+360 : angleErr;   
    	
    	return angleErr;
    }    
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	// Find angle error.  - = left, + = right
    	angleErr = getAngleError();
    	
        if (Robot.smartDashboardDebug) {
        	SmartDashboard.putNumber("Drive Angle Error", angleErr);
        }
        
    	angleTol.check(Math.abs(angleErr));
    	
    	if (angleTol.success()) {
        	Robot.driveTrain.stop();
        	Robot.writeLog("Auto turn (finish):  current angle = " + Robot.driveTrain.getDegrees() + 
        			", target angle = " + targetAngle);
    	} else {
        	// Find speed to drive
        	speedControl = angleErr*kPangle;
        	speedControl = (speedControl>commandSpeed) ? commandSpeed : speedControl;
        	speedControl = (speedControl<-commandSpeed) ? -commandSpeed : speedControl;
        	
        	if (speedControl>0) {
        		speedControl = (speedControl<minSpeed) ? minSpeed : speedControl;
        	} else {
        		speedControl = (speedControl>-minSpeed) ? -minSpeed : speedControl;
        	}
        	
        	Robot.driveTrain.driveCurve(speedControl, 1);
//        	System.out.print(commandSpeed + "  "+ speedControl);    
        	
        	priorAngleErr = angleErr;
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
