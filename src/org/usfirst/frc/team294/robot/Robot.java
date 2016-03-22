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
	Command shootBall;
	Command setFlyWheels;
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
	public static boolean smartDashboardDebug = true;		// true to print lots of stuff on the SmartDashboard
	
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

		// instantiate the command used for the autonomous period
		//autonomousCommand = new AutonomousCommandGroup();
		raiseArm90 = new ShooterArmMoveToSetLocation(90);
		
		// instantiate commands for xbox triggers
		shootBall = new ShootBall();
		setFlyWheels = new FlyWheelSetToSpeed(2100, 2520);

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
		
//		if ((intake up || intake indeterminate) && arm > 45) {
		if ((intake.intakeIsUp() || intake.intakeSolenoidIsOff()) && shooterArm.getAngle() > 45) {
			raiseArm90.start();			
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		Scheduler.getInstance().run();

        //Here is where the triggers are processed, so when they are over a certain threshold, it will run a command.
		if(oi.xboxController.getRawAxis(2) > .90){
			setFlyWheels.start();		//This one will rev the fly wheels up to 4500 RPM
		}
		if(oi.xboxController.getRawAxis(3) > .90){
			shootBall.start();			//This will do the shooting sequence
		}

		// Show arm angle
        shooterArm.updateSmartDashboard();
        
		// Other printouts
		shooter.isBallLoaded();
		intake.intakeIsUp();
		driveTrain.getDegrees();
		
		vision.getGoalXAngleError();
		
		if (smartDashboardDebug) {
			// Uncomment the following line to read coPanel knobs.
//			oi.updateSmartDashboard();

			// Uncomment the following 2 lines for debugging shooter motors PIDs.
//			shooter.setPIDFromSmartDashboard();
			shooter.updateSmartDashboard();
			
			// Uncomment the following 2 lines for debugging the arm motor PID.
//	        shooterArm.setPIDFromSmartDashboard();

			// Uncomment the following 2 lines to see drive train data
	    	driveTrain.getLeftEncoder();
	    	driveTrain.getRightEncoder();
			
			intake.updateSmartDashboard();

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