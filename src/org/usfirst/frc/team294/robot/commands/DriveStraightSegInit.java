package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.utilities.ToleranceChecker;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Drives straight a given distance using the NavX for angle correction.
 */
public class DriveStraightSegInit extends Command {

	public enum Units { rotations, inches }; 
	
	// Initial settings when command was invoked
    private double commandSpeed;
    private double distance;
    
    private boolean done;
    
    // Encoder and distance settings
//    private final double encTickPerRev = 4000;
    private double distErr, distSpeedControl;
    private double inchesPerRevolution = 18.5;
    private double minSpeed = 0.1;
    
    // Steering settings
    private double angleErr, curve;
    private double kPangle = 0.018;    //0.018
    
    /**
     * Drives straight a given distance using the NavX for angle correction and encoders for precise distance.
     * The distance is reset and starts accumulating with this segment!
     * This is the initial segment, so the robot keeps driving into the next segment.
     * @param speed +1 = full speed, 0  = don't move
     * @param distance in "units", + = forward, - = backwards
     * @param units = DriveStraightDistance.rotations or DriveStraightDistance.units
     */
    public DriveStraightSegInit(double speed, double distance, Units units) {
        commandSpeed = Math.abs(speed);
//        this.distance = distance*encTickPerRev;
        if (units == Units.rotations) {
            this.distance = distance;        	
        } else {
            this.distance = distance / inchesPerRevolution;
        }

        done = false;
        
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	// Reset distance and rotation values, since this is the first segment
    	Robot.driveTrain.resetDegrees();
    	Robot.driveTrain.resetEncoders();

    	Robot.writeLog("Drive Straight Init (init):  speed = " + commandSpeed + ", dist = " + distance);
    	Robot.writeLog("Drive Straight Init (init):  Left encoder = " + Robot.driveTrain.getLeftEncoder() + " right encoder = " + Robot.driveTrain.getRightEncoder());
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	// Find distance remaining to go
    	//distErr = ( (distance - Robot.driveTrain.getLeftEncoder()) + (distance - Robot.driveTrain.getRightEncoder())) / 2;  // average
    	// Min err is safer than average error, in case one encoder fails
    	distErr = Math.min( distance - Robot.driveTrain.getLeftEncoder(), distance - Robot.driveTrain.getRightEncoder() );

    	done = (distErr<=0);
    	if (!done) {
        	distSpeedControl = commandSpeed;
        	
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
    	} else {
        	Robot.writeLog("Drive Straight Init (finish):  Left encoder = " + Robot.driveTrain.getLeftEncoder() + " right encoder = " + Robot.driveTrain.getRightEncoder());    		
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return done;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.driveTrain.driveCurve(0, 0);
    }
}
