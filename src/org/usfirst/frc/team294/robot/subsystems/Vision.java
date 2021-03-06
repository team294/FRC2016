package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.utilities.RunningAverageFilter;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Vision extends Subsystem {
    // Local data
	boolean bGoalFound;				// Are the goal coords in xsGoal, ysGoal valid?
	boolean bGoalFoundSaved;		// Saved version of bGoalFound -- when saveGoalFoundState() is called
	double xsGoal, ysGoal;  		// Screen coords of center of largest goal
	double dGoal;					// Distance to goal, in inches
	double xsGoalTarget;			// Ideal screen location of goal center (in pixels), based on distance to goal
	double xAngleErr;				// Angle to turn robot, in degrees, to point at goal
	double ysGoalTarget;			// Ideal screen location of goal center (in pixels), based on distance to goal
	double goalArmAngle;			// Angle to set arm (in arm degrees) to point to goal
	double speedTopFlywheel;		// Top flywheel speed
	double speedBottomFlywheel;		// Bottom flywheel speed
	
	// GRIP data
	NetworkTable table;
	NetworkTable grip_table;
	double[] centerX, centerY, width;
	double[] networkTableDefault = new double[] { -1.0 };

	// Maximum flywheel speed
	final double maxFlywheelSpeed = RobotMap.maxFlywheelSpeed;	
	
	// Running average filter to smooth noise in arm data
	RunningAverageFilter armAngleFiltered = new RunningAverageFilter(19);
	
	// Calculations
//	final double hGoal = 85.0 - 2.0 + 12.25/2.0;	// Height of middle of goal -- In our lab
	final double hGoal = 85.0 - 1.5 + 12.0/2.0;	// Height of middle of goal -- Competition field
	final double hAxle = 12.5;		// Height of arm axle
	final double dArm = 20.0;			// Length of arm from axle to camera
	final double angleArmHorizontal = 7.6;	// Arm angle at horizontal, practice bot = 9.5
	final double angleArmVertical = 102.6;	// Arm angle at vertical, practice bot = 104.7
	final double angleArmM = 90.0/(angleArmVertical - angleArmHorizontal);	// Slope
	final double angleArmB = -angleArmM*angleArmHorizontal;				// Intercept
	final double cameraXRatio = 84.25/90.0/2.0*1.1;  	// Sx/Sz
	final double cameraYRatio = cameraXRatio*480.0/640.0;  	// SY/Sz
	final double cameraXHalfRes = 640/2;		// Resolution
	final double cameraYHalfRes = 480/2;		// Resolution
	final double xsBallScreen = 160;				// Ignore anything to the left of this (ball and elastic band)
	
	public Vision(){
		table = NetworkTable.getTable("GRIP/myContoursReport");
		grip_table = NetworkTable.getTable("GRIP");
		bGoalFound = false;
	}
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public void enableCameraSaving(){
		grip_table.putNumber("saveImagePeriod", 0);
		grip_table.putBoolean("saveImageEnabled", true);
	}
	
	public void disableCameraSaving(){
		grip_table.putBoolean("saveImageEnabled", false);
	}
	
	public void setCameraPeriod(double period){
		grip_table.putNumber("saveImagePeriod", period);
	}

	/**
	 * Saves to memory whether or not the last call to findGoal() found a goal.
	 */
	public void saveGoalFoundState() {
		bGoalFoundSaved = bGoalFound;
	}
	
	/**
	 * Recall the last state that was saved by saveGoalFoundState()
	 * @return true = goal was found just before saving, false = goal was not found just before saving
	 */
	public boolean readSavedGoalFoundState() {
		return bGoalFoundSaved;
	}
	
	/**
	 * Searches for goal and primes internal data structures with goal coordinates.
	 * @return True if a goal was found.
	 */
	public boolean findGoal() {
		// For finding the goal closest to "on target"
		int i;
		
		// For distance calcs
		double angleGoalOnScreen, angleArm;
		
		// Get GRIP data
		do {
			centerX = table.getNumberArray("centerX", networkTableDefault );
			centerY = table.getNumberArray("centerY", networkTableDefault );
		} while (centerX.length!=centerY.length);
			
		// Are any contours found?
		if (centerX.length>0) {
			bGoalFound = (centerX[0]>=0);
		} else {
			bGoalFound = false;
		}
		if (!bGoalFound) {
			SmartDashboard.putNumber("Cam goal distance",-1);
			return bGoalFound;
		}
		
		// Filter arm angle
		angleArm = Robot.shooterArm.getAngle();
		if (armAngleFiltered.getNumPoints()>0) {
			// If arm has moved a lot since we last averaged, then flush the average.
			if (Math.abs(armAngleFiltered.getAverage()-angleArm) > 5.0) 
				armAngleFiltered.reset();
		}
		armAngleFiltered.add(angleArm);
		angleArm = armAngleFiltered.getAverage()*angleArmM + angleArmB;
		
		double dGoalTemp, xsGoalTargetTemp;
		
		// Find goal closest to "on target" in X direction.  
		bGoalFound = false;
		for (i=0; i<centerX.length; i++) {
			// Filter out ball and elastic band (X <= xsBallSceen)
			if (centerX[i]>xsBallScreen) {
				// Calculate horizontal distance to goal, in inches
				angleGoalOnScreen = Math.atan( (cameraYHalfRes - centerY[i])/cameraYHalfRes * cameraYRatio)*180.0/Math.PI;
				dGoalTemp = (hGoal - hAxle - dArm*Math.sin(angleArm*Math.PI/180.0)) / Math.tan((angleArm + angleGoalOnScreen)*Math.PI/180.0);				

				// Calculate ideal screen X position of goal
//				if (dGoalTemp<100) {
//					xsGoalTargetTemp = -0.00469436*dGoalTemp*dGoalTemp + 1.170263*dGoalTemp + 206.3622;
//					xsGoalTargetTemp = -0.00469436*dGoalTemp*dGoalTemp + 1.170263*dGoalTemp + 184.4;
//					xsGoalTargetTemp = -0.007805*dGoalTemp*dGoalTemp + 1.5009*dGoalTemp + 188.0;
//				} else {
//					xsGoalTargetTemp = 188.0;
//				}
				xsGoalTargetTemp = 0.171429*dGoalTemp + 219.7;  // Last few qualifier matches
//				xsGoalTargetTemp = 0.19588*dGoalTemp + 220.4;  // Based on recalibration on practice field.  Targets ~0.3 deg to the left of prior code.
				
				// If this is the first goal or closer than the last goal, then save it
				if (!bGoalFound || ( Math.abs(xsGoalTargetTemp-centerX[i])<Math.abs(xsGoalTarget-xsGoal) )) {
					bGoalFound=true;
					dGoal = dGoalTemp;
					xsGoalTarget = xsGoalTargetTemp;
					xsGoal = centerX[i];
					ysGoal = centerY[i];
				}
			}
		}
		
		if (!bGoalFound) {
			SmartDashboard.putNumber("Cam goal distance",-2);
			return bGoalFound;			
		}
		
		SmartDashboard.putNumber("Cam goal distance",dGoal);
		
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
//			goalArmAngle = 0.00207*dGoal*dGoal -0.5694*dGoal + 93.1;			
//			goalArmAngle = 0.00234*dGoal*dGoal -0.6629*dGoal + 96.1;			
//			goalArmAngle = 0.00196*dGoal*dGoal -0.5434*dGoal + 88.4;			
			goalArmAngle = 0.00196*dGoal*dGoal -0.5434*dGoal + 87.4;	// Lower goal angle 1 degree -- shooting slightly high with new balls in quarter finals
		} else {
//			goalArmAngle = 54.0;
//			goalArmAngle = 49.0;
//			goalArmAngle = 51.0;  
			goalArmAngle = 50.0;  // Lower goal angle 1 degree -- shooting slightly high with new balls in quarter finals
		}
		
		// Calculate flywheel speeds
		if (dGoal<=50) {
			speedTopFlywheel = 2100.0;
			speedBottomFlywheel = 2520.0;
		} else  {
			speedTopFlywheel = maxFlywheelSpeed;
			speedBottomFlywheel = maxFlywheelSpeed;
//		} else {
//			speedTopFlywheel = 75.0*dGoal - 900.0;
//			speedBottomFlywheel = 61.88*dGoal + 45;
//			speedTopFlywheel = (speedTopFlywheel>maxFlywheelSpeed) ? maxFlywheelSpeed : speedTopFlywheel;
//			speedBottomFlywheel = (speedBottomFlywheel>maxFlywheelSpeed) ? maxFlywheelSpeed : speedBottomFlywheel;			
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

	/**
	 * Returns the ideal speed for the top flywheel to target the last goal found by findGoal()
	 * @return top flywheel speed, in rpm.  MaxFlywheelSpeed if no goal found.
	 */
	public double getTopFlywheelSpeed() {
		if (!bGoalFound) {
			SmartDashboard.putNumber("Cam top flywheel", maxFlywheelSpeed);
			return maxFlywheelSpeed;
		}
		
		SmartDashboard.putNumber("Cam top flywheel", speedTopFlywheel);
		return speedTopFlywheel;		
	}
	
	/**
	 * Returns the ideal speed for the bottom flywheel to target the last goal found by findGoal()
	 * @return bottom flywheel speed, in rpm.  MaxFlywheelSpeed if no goal found.
	 */
	public double getBottomFlywheelSpeed() {
		if (!bGoalFound) {
			SmartDashboard.putNumber("Cam bottom flywheel", maxFlywheelSpeed);
			return maxFlywheelSpeed;
		}
		
		SmartDashboard.putNumber("Cam bottom flywheel", speedBottomFlywheel);
		return speedBottomFlywheel;		
	}
	
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

