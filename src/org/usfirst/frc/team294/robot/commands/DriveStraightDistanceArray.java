package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.utilities.ToleranceChecker;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Drives straight a given distance using the NavX for angle correction.
 */
public class DriveStraightDistanceArray extends Command {

	public enum Units { rotations, inches }; 
	
	// Initial settings when command was invoked
	private double[] speedArr, distanceArr;
	private Units units;
	
	// Settings at each step
	private boolean lastSegment;
	private int idx;
    private double commandSpeed;
    private double distance;
    
    // Encoder and distance settings
//    private final double encTickPerRev = 4000;
    private double distErr, distSpeedControl;
    private double kPdist = 3;
    private double inchesPerRevolution = 18.5;
    private double minSpeed = 0.1;
    
    // Steering settings
    private double angleErr, curve;
    private double kPangle = 0.018;    //0.018
    
    // Check if target has been reached
//    ToleranceChecker driveTol = new ToleranceChecker(100, 5);
    ToleranceChecker driveTol = new ToleranceChecker(0.02, 5);
    
    
    //TODO:  Test this command!
    /**
     * THIS CODE NOT TESTED!!!
     * Drives straight a sequence of given distances/speeds using the NavX for angle correction.
     * The distances add up cumulatively, but only the total distance is precise.  The distance
     * and speed arrays must have the same length (command terminates at the shorter of the two arrays).
     * @param speed[] +1 = full speed, 0  = don't move.  Should never be negative.
     * @param distance[] in "units", + = forward, - = backwards
     * @param units = DriveStraightDistance.rotations or DriveStraightDistance.units <p>
     * Example:  ({1.0, 0.6, 1.0, 0.4}, {2*12, 6*12, 2*12, 1*12}, Units.inches)
     * drives 2 ft (fast), 6 ft (slow), 2 ft (fast), 1 ft (slow)
     */
    public DriveStraightDistanceArray(double[] speed, double[] distance, Units units) {
    	speedArr = speed;
    	distanceArr = distance;
    	this.units = units;
    	        
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
//        distance = SmartDashboard.getNumber("Go rotations", 1.0);

    	driveTol.reset();
    	Robot.driveTrain.resetDegrees();
    	Robot.driveTrain.resetEncoders();
    	angleErr = 0;
    	distSpeedControl = 1;

    	idx = 0;
    	if (speedArr.length==0 || distanceArr.length==0) {
    		commandSpeed = 0;
    		distance = 0;
    		lastSegment = true;
    		return;
    	}
    	commandSpeed = Math.abs(speedArr[idx]);
    	if (units == Units.rotations) {
    		distance = distanceArr[idx];        	
    	} else {
    		distance = distanceArr[idx] / inchesPerRevolution;
    	}
    	lastSegment = (speedArr.length<=(idx+1) || distanceArr.length<=(idx+1));
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	// Find distance remaining to go
    	//distErr = ( (distance - Robot.driveTrain.getLeftEncoder()) + (distance - Robot.driveTrain.getRightEncoder())) / 2;  // average
    	// Min err is safer than average error, in case one encoder fails
    	distErr = Math.min( distance - Robot.driveTrain.getLeftEncoder(), distance - Robot.driveTrain.getRightEncoder() );
    	
        if (Robot.smartDashboardDebug) {
        	SmartDashboard.putNumber("Drive Straight Error", distErr);
        }
        
        if (lastSegment) {
        	// last segment, so use proportional control
            driveTol.check(Math.abs(distErr));        	

        	// Find speed to drive
        	distSpeedControl = distErr*kPdist;
        	distSpeedControl = (distSpeedControl>1) ? 1 : distSpeedControl;
        	distSpeedControl = (distSpeedControl<-1) ? -1 : distSpeedControl;
        	distSpeedControl *= commandSpeed;
        } else {
        	// Currently not last segment
        	if (distErr<0) {
        		// If distErr<0, then go to next segment
            	idx++;
            	commandSpeed = Math.abs(speedArr[idx]);
            	if (units == Units.rotations) {
            		distance += distanceArr[idx];
            		distErr += distanceArr[idx];
            	} else {
            		distance += distanceArr[idx] / inchesPerRevolution;
            		distErr += distanceArr[idx] / inchesPerRevolution;
            	}
            	lastSegment = (speedArr.length<=(idx+1) || distanceArr.length<=(idx+1));
        	}
        	
            if (lastSegment) {
            	// We are now at last segement, so use proportional control
                driveTol.check(Math.abs(distErr));        	

            	// Find speed to drive
            	distSpeedControl = distErr*kPdist;
            	distSpeedControl = (distSpeedControl>1) ? 1 : distSpeedControl;
            	distSpeedControl = (distSpeedControl<-1) ? -1 : distSpeedControl;
            	distSpeedControl *= commandSpeed;
            } else {
            	// Not last segement.  Drive at full commandSpeed.  
            	// If we overshoot this segment, no big deal, distance is part of next segment.
            	distSpeedControl = commandSpeed;        	            	
            }
        }
    	
    	if (!driveTol.success()) {
        	// Use minSpeed to stay out of dead band
        	if (distSpeedControl>0) {
        		distSpeedControl = (distSpeedControl<minSpeed) ? minSpeed : distSpeedControl;
        	} else {
        		distSpeedControl = (distSpeedControl>-minSpeed) ? -minSpeed : distSpeedControl;
        	}
        	
        	// Find angle to drive
        	angleErr = Robot.driveTrain.getDegrees();
        	angleErr = (angleErr>180) ? angleErr-360 : angleErr;
        	angleErr = (Math.abs(angleErr) <= 10.0) ? angleErr : 0.0;		// Assume if we are more than 10 deg off then we have a NavX failure
        	
            curve = angleErr*kPangle;
        	curve = (curve>0.5) ? 0.5 : curve;
        	curve = (curve<-0.5) ? -0.5 : curve;
        	curve = (distErr>=0) ? -curve : curve; // Flip sign if we are going forwards
        	
        	Robot.driveTrain.driveCurve(distSpeedControl, curve);
//        	System.out.print(commandSpeed + "  "+ distSpeedControl+"  "+curve);    		
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return driveTol.success();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.driveCurve(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.driveTrain.driveCurve(0, 0);
    }
}
