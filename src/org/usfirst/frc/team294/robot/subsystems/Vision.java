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
		int i, goal;
		double maxWidth;
		
		width = table.getNumberArray("width", networkTableDefault );
		centerX = table.getNumberArray("centerX", networkTableDefault );
		centerY = table.getNumberArray("centerY", networkTableDefault );

		// Are any contours found?
		if (width.length>0) {
			bGoalFound = (width[0]>=0);
		} else bGoalFound = false;

		// Find goal with largest width
		if (bGoalFound) {
			maxWidth = 0;
			goal = 0;
			for (i=0; i<width.length; i++) {
				if (width[i]>maxWidth) {
					maxWidth = width[i];
					goal = i;
				}
			}
			
			xsGoal = centerX[goal];
			ysGoal = centerY[goal];
		}
		
		return bGoalFound;
	}
	
	/**
	 * Returns the horizontal distance to last goal found with findGoal(), in inches
	 * @return distance, in inches.  -1 if no goal found.
	 */
	public double getGoalDistance(){
		double angleGoalOnScreen, angleArm;
		double dGoal;
		
		if (bGoalFound) {
			angleGoalOnScreen = Math.atan( (cameraYHalfRes - ysGoal)/cameraYHalfRes * cameraYRatio)*180.0/Math.PI;
			angleArm = Robot.shooterArm.getAngle()*angleArmM + angleArmB;
			dGoal = (hGoal - hAxle - dArm*Math.sin(angleArm*Math.PI/180.0)) / Math.tan((angleArm + angleGoalOnScreen)*Math.PI/180.0);				
		} else dGoal = -1;
		
		SmartDashboard.putNumber("Cam goal distance",dGoal);
		return dGoal;
    }

	/**
	 * Returns the X angle error (in degrees) to the last goal found by findGoal()
	 * @return error, in degrees (-=left, +=right).  0 if no goal found.
	 */
	public double getGoalXAngleError(){
		double dGoal, dXGoalTarget;
		double angleErr;
		
		if (!bGoalFound) return 0;

		dGoal = getGoalDistance();
		
		if (dGoal<100) {
			dXGoalTarget = -0.00469436*dGoal*dGoal + 1.170263*dGoal + 217.3622;
		} else {
			dXGoalTarget = 287.0;
		}
		SmartDashboard.putNumber("Cam goal X pixel target", dXGoalTarget);
//		dXGoalTarget = 287.0;		// Override distance calcs for now (need to debug)
		SmartDashboard.putNumber("Cam goal X pixel target2", dXGoalTarget);
		SmartDashboard.putNumber("Cam goal X center", xsGoal);

		angleErr = Math.atan( (xsGoal-dXGoalTarget)/cameraXHalfRes * cameraXRatio) * 180.0/Math.PI;
		SmartDashboard.putNumber("Cam goal X angle err", angleErr);
		
		return angleErr;
	}
	
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

