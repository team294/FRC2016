package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.utilities.ToleranceChecker;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Drives straight a given distance using the NavX for angle correction.
 */
public class DriveStraightDistance extends Command {

	// Initial settings when command was invoked
    private double commandSpeed;
    private double distance;
    
    // Encoder and distance settings
//    private final double encTickPerRev = 4000;
    private double distErr, distSpeedControl;
    private double kPdist = 3;
    
    // Steering settings
    private double angleErr, curve;
    private double kPangle = 0.05;
    
    // Check if target has been reached
//    ToleranceChecker driveTol = new ToleranceChecker(100, 5);
    ToleranceChecker driveTol = new ToleranceChecker(0.02, 5);
    
    /**
     * Drives straight a given distance using the NavX for angle correction.
     * @param speed +1 = full forward, -1 = full reverse
     * @param distance in wheel revolutions
     */
    public DriveStraightDistance(double speed, double distance) {
        commandSpeed = speed;
//        this.distance = distance*encTickPerRev;
        this.distance = distance;
        
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
    	curve = -angleErr*kPangle;
    	curve = (curve>1) ? 1 : curve;
    	curve = (curve<-1) ? -1 : curve;
    	
    	// Find speed to drive
    	distErr = ( (distance - Robot.driveTrain.getLeftEncoder()) + (distance - Robot.driveTrain.getRightEncoder())) / 2;
    	distSpeedControl = distErr*kPdist;
    	distSpeedControl = (distSpeedControl>1) ? 1 : distSpeedControl;
    	distSpeedControl = (distSpeedControl<-1) ? -1 : distSpeedControl;
    	Robot.driveTrain.driveCurve(commandSpeed*distSpeedControl, curve);
//    	System.out.print(commandSpeed + "  "+ distSpeedControl+"  "+curve);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return driveTol.success(Math.abs(distErr));
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
