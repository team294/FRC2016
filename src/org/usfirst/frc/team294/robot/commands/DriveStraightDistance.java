package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.utilities.ToleranceChecker;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Drives straight a given distance using the NavX for angle correction.
 */
public class DriveStraightDistance extends Command {

	public enum Units { rotations, inches }; 
	
	// Initial settings when command was invoked
    private double commandSpeed;
    private double distance;
    
    // Encoder and distance settings
//    private final double encTickPerRev = 4000;
    private double distErr, distSpeedControl;
    private double kPdist = 3;
    private double inchesPerRevolution = 18.5;
    private double minSpeed = 0.1;
    
    // Steering settings
    private double angleErr, curve, sign;
    private double kPangle = 0.018; 
    
    // Check if target has been reached
//    ToleranceChecker driveTol = new ToleranceChecker(100, 5);
    ToleranceChecker driveTol = new ToleranceChecker(0.02, 5);
    
    /**
     * Drives straight a given distance using the NavX for angle correction.
     * @param speed +1 = full speed, 0  = don't move
     * @param distance in "units", + = forward, - = backwards
     * @param units = DriveStraightDistance.rotations or DriveStraightDistance.units
     */
    public DriveStraightDistance(double speed, double distance, Units units) {
        commandSpeed = Math.abs(speed);
//        this.distance = distance*encTickPerRev;
        if (units == Units.rotations) {
            this.distance = distance;        	
        } else {
            this.distance = distance / inchesPerRevolution;
        }
        if (distance>=0) {
        	sign = -1.0;
        } else {
        	sign = 1.0;
        }
        
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
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	// Find angle to drive
    	angleErr = Robot.driveTrain.getDegrees();
    	angleErr = (angleErr>180) ? angleErr-360 : angleErr;
    	curve = sign*angleErr*kPangle;
    	curve = (curve>1) ? 1 : curve;
    	curve = (curve<-1) ? -1 : curve;
    	
    	// Find speed to drive
    	distErr = ( (distance - Robot.driveTrain.getLeftEncoder()) + (distance - Robot.driveTrain.getRightEncoder())) / 2;
    	SmartDashboard.putNumber("Left Error", distErr);
    	driveTol.check(Math.abs(distErr));
    	
    	if (!driveTol.success()) {
        	distSpeedControl = distErr*kPdist;
        	distSpeedControl = (distSpeedControl>1) ? 1 : distSpeedControl;
        	distSpeedControl = (distSpeedControl<-1) ? -1 : distSpeedControl;
        	distSpeedControl *= commandSpeed;
        	
        	if (distSpeedControl>0) {
        		distSpeedControl = (distSpeedControl<minSpeed) ? minSpeed : distSpeedControl;
        	} else {
        		distSpeedControl = (distSpeedControl>-minSpeed) ? -minSpeed : distSpeedControl;
        	}
        	
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
