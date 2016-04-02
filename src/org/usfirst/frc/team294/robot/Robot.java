package org.usfirst.frc.team294.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team294.robot.commands.*;
import org.usfirst.frc.team294.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	Command autonomousCommand;
	Command raiseArm90;
	
	public static OI oi;

	// Creates the SubSystem onjects
	public static DriveTrain driveTrain;
	public static Shifter shifter;
	public static ShooterArm shooterArm;
	public static Shooter shooter;
	public static Intake intake;
	public static Vision vision;

	public static PowerDistributionPanel panel;

	// Turn on/off SmartDashboard debugging
	public static boolean smartDashboardDebug = false;		// true to print lots of stuff on the SmartDashboard
	
	public static boolean overrideIntake;
	
	//for preferences armMin position, arm 90 degree position
	Preferences prefs;
	public static double armCalMinPosition;
	public static double armCal90DegPosition;
	public static boolean shooterArmEnabled;		// Safety disable arm if parameters are missing


	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		RobotMap.init();

		//Get preferences from robot flash memory
		prefs = Preferences.getInstance();
		armCalMinPosition = prefs.getDouble("armCalMinPosition", 0);
		armCal90DegPosition = prefs.getDouble("armCal90DegPosition", 0);
		if (armCalMinPosition==0 || armCal90DegPosition==0) {
			DriverStation.reportError("Error:  Preferences missing from RoboRio for Shooter Arm position calibration.  Shooter arm disabled.", true);
			shooterArmEnabled = false;
		} else {
			shooterArmEnabled = true;
		}

		//Instantiates the subsystems
		driveTrain = new DriveTrain();
		shifter = new Shifter();
		shooterArm = new ShooterArm();
		shooter = new Shooter();
		intake = new Intake();
		vision = new Vision();
		panel = new PowerDistributionPanel();

		// OI must be constructed after subsystems. If the OI creates Commands
		// (which it very likely will), subsystems are not guaranteed to be
		// constructed yet. Thus, their requires() statements may grab null
		// pointers. Bad news. Don't move it.
		oi = new OI();

        intake.motorCurrentTrigger.whenActive(new IntakeMotorStop());

		// instantiate the command used for the autonomous period
		//autonomousCommand = new AutonomousCommandGroup();
		raiseArm90 = new ShooterArmMoveToSetLocation(90);
		
		// Display active commands and subsystem status on SmartDashboard
		SmartDashboard.putData(Scheduler.getInstance());
		SmartDashboard.putData(driveTrain);
		SmartDashboard.putData(shifter);
		SmartDashboard.putData(shooterArm);
		SmartDashboard.putData(shooter);
		SmartDashboard.putData(intake);
		SmartDashboard.putData(vision);
	}

	/**
	 * This function is called when the disabled button is hit. You can use it
	 * to reset subsystems before shutting down.
	 */
	public void disabledInit() {

	}

	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	public void autonomousInit() {
		// schedule the autonomous command (example)
		autonomousCommand = oi.getAutonomousCommand();
		
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
		
//		if ((intake.intakeIsUp() || intake.intakeSolenoidIsOff()) && shooterArm.getAngle() > 45) {
//			raiseArm90.start();			
//		}
		shooterArm.setBrakeOff();
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		Scheduler.getInstance().run();

		// Show arm angle
        shooterArm.updateSmartDashboard();
        
		// Other printouts
		shooter.updateSmartDashboard();
		shooter.isBallLoaded();
		intake.updateSmartDashboard();
		intake.intakeIsUp();
		driveTrain.getDegrees();
		
		vision.findGoal();
		vision.getGoalXAngleError();
		vision.getGoalArmAngle();
		
		if (smartDashboardDebug) {
			// Uncomment the following line to read coPanel knobs.
//			oi.updateSmartDashboard();

			// Uncomment the following line for debugging shooter motors PIDs.
//			shooter.setPIDFromSmartDashboard();
			
			// Uncomment the following line for debugging the arm motor PID.
//	        shooterArm.setPIDFromSmartDashboard();
			
			// Uncomment the following line to see drive train data
	    	driveTrain.getLeftEncoder();
	    	driveTrain.getRightEncoder();
			
			//		SmartDashboard.putNumber("Panel voltage", panel.getVoltage());
			//		SmartDashboard.putNumber("Panel arm current", panel.getCurrent(0));
		}

	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
	}
}