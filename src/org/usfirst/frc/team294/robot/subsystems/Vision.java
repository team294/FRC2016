package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Vision extends Subsystem {
    
	// GRIP data
	NetworkTable table;
	double[] centerX, centerY;
	double[] networkTableDefault = new double[] { -1.0 };
	
	// Calculations
	final double hGoal = 85.0 - 2.0 + 12.25/2;	// Height of middle of goal
	final double hAxle = 12.5;		// Height of arm axle
	final double dArm = 20.0;			// Length of arm from axle to camera
	final double angleArmHorizontal = 7.75;	// Arm angle at horizontal
	final double angleArmVertical = 103.0;	// Arm angle at vertical
	final double angleArmM = 90.0/(angleArmVertical - angleArmHorizontal);	// Slope
	final double angleArmB = -angleArmM*angleArmHorizontal;				// Intercept
	final double cameraXRatio = 84.25/90.0/2.0*1.1;  	// Sx/Sz
	final double cameraYRatio = cameraXRatio*480.0/640.0;  	// SY/Sz
	final double cameraXHalfRes = 640/2;		// Resolution
	final double cameraYHalfRes = 480/2;		// Resolution
	
	public Vision(){
		table = NetworkTable.getTable("GRIP/myContoursReport"); 
	}
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	/**
	 * Returns the horizontaal distance to goal in contour[0], in inches
	 * @return distance, in inches.  -1 if no goal found.
	 */
	public double getGoalDistance(){
		double angleGoalOnScreen, angleArm;
		double dGoal;
		
		centerY = table.getNumberArray("centerY", networkTableDefault );
		
		if (centerY.length>0) {
			if (centerY[0]>=0) {
				angleGoalOnScreen = Math.atan( (cameraYHalfRes - centerY[0])/cameraYHalfRes * cameraYRatio);
				angleArm = Robot.shooterArm.getAngle()*angleArmM + angleArmB;
				dGoal = (hGoal - hAxle - dArm*Math.sin(angleArm*Math.PI/180.0)) / Math.tan((angleArm + angleGoalOnScreen)*Math.PI/180.0);				
			} else dGoal = -1;
		} else dGoal = -1;
		
		SmartDashboard.putNumber("Cam goal distance",dGoal);
		return dGoal;
    }

	/**
	 * Returns the X angle error to the goal, in degrees 
	 * @return error, in degrees (-=left, +=right).  0 if no goal found.
	 */
	public double getGoalXAngleError(){
		double dGoal, dXGoalTarget;
		double angleErr;
		
		dGoal = getGoalDistance();
		
		centerX = table.getNumberArray("centerX", networkTableDefault );
		if (centerX.length==0) return 0;
		if (centerX[0]<0) return 0;
		
		if (dGoal<100) {
			dXGoalTarget = -0.00469436*dGoal*dGoal + 1.170263*dGoal + 217.3622;
		} else {
			dXGoalTarget = 287.0;
		}
		SmartDashboard.putNumber("Cam goal X pixel target", dXGoalTarget);
		dXGoalTarget = 287.0;		// Override distance calcs for now (need to debug)
		SmartDashboard.putNumber("Cam goal X pixel target2", dXGoalTarget);
		SmartDashboard.putNumber("Cam goal X center", centerX[0]);

		angleErr = Math.atan( (centerX[0]-dXGoalTarget)/cameraXHalfRes * cameraXRatio) * 180.0/Math.PI;
		SmartDashboard.putNumber("Cam goal X angle err", angleErr);
		
		return angleErr;
	}
	
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

