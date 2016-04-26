package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.utilities.ToleranceChecker;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTurnToGoal extends DriveAngle {

    /** 
     * Turns robot to face largest goal found, to within tolerance
     * @param angleTolerance = accuracy of robot turning which is good enough to stop command, in degrees
     */
    public DriveTurnToGoal(double angleTolerance) {
        super(0.65, 0, true, angleTolerance);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.vision.findGoal();
        targetAngle = Robot.vision.getGoalXAngleError();

    	System.out.println("Auto turn to goal:  Robot angle before reset = " + Robot.driveTrain.getDegrees() + 
    			", target angle = " + targetAngle);
        
        targetAngle = (targetAngle < 0) ? targetAngle+360.0 : targetAngle;
       	super.initialize();
//    	angleTol.reset();
//    	angleTol.setTolerance(maxTol);
//       	Robot.driveTrain.resetDegrees();    		
       	
    	System.out.println("Auto turn to goal:  dist = " + Robot.vision.getGoalDistance() + ", target angle = " 
    			+ targetAngle + ", current angle = " + Robot.driveTrain.getDegrees());
    }
}
