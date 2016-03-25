package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Vision extends Subsystem {
    // Local data
	boolean bGoalFound;				// Are the goal coords in xsGoal, ysGoal valid?
	double xsGoal, ysGoal;  		// Screen coords of center of largest goal
	double dGoal;					// Distance to goal, in inches
	double xsGoalTarget;			// Ideal screen location of goal center (in pixels), based on distance to goal
	double xAngleErr;				// Angle to turn robot, in degrees, to point at goal
	double ysGoalTarget;			// Ideal screen location of goal center (in pixels), based on distance to goal
	double goalArmAngle;			// Angle to set arm (in arm degrees) to point to goal

	
	// GRIP data
	NetworkTable table;
	double[] centerX, centerY, width;
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
	final double xsBallScreen = 150;				// Ignore anything to the left of this (ball and elastic band)
	
	public Vision(){
		table = NetworkTable.getTable("GRIP/myContoursReport");
		bGoalFound = false;
	}
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	/**
	 * Searches for goal and primes internal data structures with goal coordinates.
	 * @return True if a goal was found.
	 */
	public boolean findGoal() {
		// For finding the goal with the correct width
		int i, goal;
		double maxWidth;
		
		// For distance calcs
		double angleGoalOnScreen, angleArm;
		
		// Get GRIP data
		width = table.getNumberArray("width", networkTableDefault );
		centerX = table.getNumberArray("centerX", networkTableDefault );
		centerY = table.getNumberArray("centerY", networkTableDefault );

		// Are any contours found?
		if (width.length>0) {
			bGoalFound = (width[0]>=0);
		} else {
			bGoalFound = false;
			SmartDashboard.putNumber("Cam goal distance",-1);
			return bGoalFound;
		}

		// Find goal with largest width.  Filter out ball and elastic band (X <= xsBallSceen)
		maxWidth = 0;
		goal = -1;
		for (i=0; i<width.length; i++) {
			if (width[i]>maxWidth && centerX[i]>xsBallScreen) {
				maxWidth = width[i];
				goal = i;
			}
		}
		
		if (goal==-1) {
			bGoalFound = false;
			SmartDashboard.putNumber("Cam goal distance",-2);
			return bGoalFound;			
		}
		
		bGoalFound = true;
		xsGoal = centerX[goal];
		ysGoal = centerY[goal];
		
		// Calculate horizontal distance to goal, in inches
		angleGoalOnScreen = Math.atan( (cameraYHalfRes - ysGoal)/cameraYHalfRes * cameraYRatio)*180.0/Math.PI;
		angleArm = Robot.shooterArm.getAngle()*angleArmM + angleArmB;
		dGoal = (hGoal - hAxle - dArm*Math.sin(angleArm*Math.PI/180.0)) / Math.tan((angleArm + angleGoalOnScreen)*Math.PI/180.0);				
		SmartDashboard.putNumber("Cam goal distance",dGoal);

		// Calculate ideal screen X position of goal
		if (dGoal<100) {
			xsGoalTarget = -0.00469436*dGoal*dGoal + 1.170263*dGoal + 217.3622;
		} else {
			xsGoalTarget = 287.0;
		}

		// Calculate angle to turn robot to point at goal
		xAngleErr = Math.atan( (xsGoal-xsGoalTarget)/cameraXHalfRes * cameraXRatio) * 180.0/Math.PI;

		// Calculate ideal screen Y position of goal
		if (dGoal<=72) {
			ysGoalTarget = -0.638*dGoal + 350;
		} else {
			ysGoalTarget = 1.279*dGoal + 209;
		}

		// Calculate arm angle to target goal
		if (dGoal<151) {
			goalArmAngle = 0.00213*dGoal*dGoal -0.6312*dGoal + 95.7;			
		} else {
			goalArmAngle = 49.0;
		}
		
		return bGoalFound;
	}
	
	/**
	 * Returns the horizontal distance to last goal found with findGoal(), in inches
	 * @return distance, in inches.  -1 if no goal found.
	 */
	public double getGoalDistance(){
		if (bGoalFound) {
			return dGoal;
		} else {
			return -1;
		}
    }

	/**
	 * Returns the X angle error (in degrees) to the last goal found by findGoal()
	 * @return error, in degrees (-=left, +=right).  0 if no goal found.
	 */
	public double getGoalXAngleError(){
		
		if (!bGoalFound) return 0;

		SmartDashboard.putNumber("Cam goal X pixel target", xsGoalTarget);
		SmartDashboard.putNumber("Cam goal X center", xsGoal);
		SmartDashboard.putNumber("Cam goal X angle err", xAngleErr);
		
		return xAngleErr;
	}
	
    
	/**
	 * Returns the ideal Arm angle to target the last goal found by findGoal()
	 * @return arm angle, in degrees.  Current arm angle if no goal found.
	 */
	public double getGoalArmAngle(){
		if (!bGoalFound) return Robot.shooterArm.getAngle();

		SmartDashboard.putNumber("Cam goal Y pixel target", ysGoalTarget);
		SmartDashboard.putNumber("Cam goal Y center", ysGoal);
		SmartDashboard.putNumber("Cam goal Y arm angle", goalArmAngle);
		
		return goalArmAngle;
	}

	
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

