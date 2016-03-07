package org.usfirst.frc.team294.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
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

	public static OI oi;
	// Creates the SubSystem onjects
	public static DriveTrain driveTrain;
	public static Shifter shifter;
	public static ShooterArm shooterArm;
	public static Shooter shooter;
	public static Intake intake;
	public static Vision vision;
	
	//for preferences armMin position, arm 90 degree position
	Preferences prefs;
	public static double armCalMinPosition;
	public static double armCal90DegPosition;
	public static boolean shooterArmEnabled;
	
	public static PowerDistributionPanel panel;
	
	
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

		shooter.setupSmartDashboard(true);
		
		// OI must be constructed after subsystems. If the OI creates Commands
		// (which it very likely will), subsystems are not guaranteed to be
		// constructed yet. Thus, their requires() statements may grab null
		// pointers. Bad news. Don't move it.
		oi = new OI();

		// instantiate the command used for the autonomous period

		autonomousCommand = new AutonomousCommandGroup();
		shootBall = new ShootBall();
		setFlyWheels = new FlyWheelSetToSpeed(4500);
		
		// Display active commands and subsystem status on SmartDashboard
		SmartDashboard.putData(Scheduler.getInstance());
		SmartDashboard.putData(driveTrain);
		SmartDashboard.putData(shifter);
		SmartDashboard.putData(shooterArm);
		SmartDashboard.putData(shooter);
		SmartDashboard.putData(intake);
		SmartDashboard.putData(vision);
		
		panel = new PowerDistributionPanel();
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
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
		// Uncomment the following 2 lines for debugging shooter motors PIDs.
//		shooter.setPIDFromSmartDashboard();
		shooter.updateSmartDashboard();
//        shooterArm.setPIDFromSmartDashboard();
        shooterArm.updateSmartDashboard();
        
        //Here is where the triggers are processed, so when they are over a certain threshold, it will run a command.
		if(oi.xboxController.getRawAxis(2) > .90){
			setFlyWheels.start();		//This one will rev the fly wheels up to 4500 RPM
		}
		if(oi.xboxController.getRawAxis(3) > .90){
			shootBall.start();			//This will do the shooting sequence
		}
		
		
        
        //This code here is the code for changing the angle using the TOP of the 3 knobs.  The switch next to it
        //applies the angle to the robot.  This code SHOULD GO IN A COMMAND.  I created it here as a concept.
//		double ballAxis = OI.coPanel.getRawAxis(4);
//		
//		double pos0 = 0.0;
//		double pos1 = 0.007874015748031496;
//		double pos2 = 0.015748031496062992;
//		double pos3 = 0.031496062992125984;
//		double pos4 = 0.03937007874015748;
//		double pos5 = 0.05511811023622047;
//		double pos6 = 0.06299212598425197;
//		double pos7 = 0.07874015748031496;
//		double pos8 = 0.08661417322834646;
//		double pos9 = 0.10236220472440945;
//		double pos10 = 0.11023622047244094;
//		double pos11 = 0.11811023622047244;
//		
//		if(ballAxis == 0){
//			System.out.println(0);
//		}else if(ballAxis > pos0 && ballAxis < pos2){
//			System.out.println(1);
//		}else if(ballAxis > pos1 && ballAxis < pos3){
//			System.out.println(2);
//		}else if(ballAxis > pos2 && ballAxis < pos4){
//			System.out.println(3);
//		}else if(ballAxis > pos3 && ballAxis < pos5){
//			System.out.println(4);
//		}else if(ballAxis > pos4 && ballAxis < pos6){
//			System.out.println(5);
//		}else if(ballAxis > pos5 && ballAxis < pos7){
//			System.out.println(6);
//		}else if(ballAxis > pos6 && ballAxis < pos8){
//			System.out.println(7);
//		}else if(ballAxis > pos7 && ballAxis < pos9){
//			System.out.println(8);
//		}else if(ballAxis > pos8 && ballAxis < pos10){
//			System.out.println(9);
//		}else if(ballAxis > pos9 && ballAxis < pos11){
//			System.out.println(10);
//		}else if(ballAxis > pos10 && ballAxis < 1){
//			System.out.println(11);
//		}

		
		// Other printouts
//        shooter.isBallLoaded();
//		intake.intakeIsUp();
		
		// Uncomment the following 2 lines to see drive train data
    	driveTrain.getLeftEncoder();
    	driveTrain.getRightEncoder();

//		SmartDashboard.putNumber("Panel voltage", panel.getVoltage());
//		SmartDashboard.putNumber("Panel arm current", panel.getCurrent(0));
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
	}
}
