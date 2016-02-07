// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc294.robot;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	//Creates the onjects for everything
    public static CANTalon driveTrainLeftMotor1;
    public static CANTalon driveTrainLeftMotor2;
    public static CANTalon driveTrainRightMotor1;
    public static CANTalon driveTrainRightMotor2;
    public static RobotDrive driveTrainRobotDrive;
    public static AnalogGyro driveTrainGyro1;
    public static DoubleSolenoid shifterSolenoid;
    public static CANTalon shooterMotor;
    public static Solenoid shooterPiston;
    

    public static void init() {
        // Instantiates all objects
        driveTrainLeftMotor1 = new CANTalon(5);
        LiveWindow.addActuator("DriveTrain", "leftMotor1", driveTrainLeftMotor1);
        
        driveTrainLeftMotor2 = new CANTalon(6);
        driveTrainLeftMotor2.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        LiveWindow.addActuator("DriveTrain", "leftMotor2", driveTrainLeftMotor2);
        
        driveTrainRightMotor1 = new CANTalon(11);
        LiveWindow.addActuator("DriveTrain", "rightMotor1", driveTrainRightMotor1);
        
        driveTrainRightMotor2 = new CANTalon(12);
        driveTrainRightMotor2.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        LiveWindow.addActuator("DriveTrain", "rightMotor2", driveTrainRightMotor2);
        
        driveTrainRobotDrive = new RobotDrive(driveTrainLeftMotor1, driveTrainLeftMotor2,
              driveTrainRightMotor1, driveTrainRightMotor2);
        driveTrainRobotDrive.setSafetyEnabled(true);
        driveTrainRobotDrive.setExpiration(0.1);
        driveTrainRobotDrive.setSensitivity(0.5);
        driveTrainRobotDrive.setMaxOutput(1.0);

        driveTrainGyro1 = new AnalogGyro(0);
        LiveWindow.addSensor("DriveTrain", "gyro1", driveTrainGyro1);
        driveTrainGyro1.setSensitivity(0.007);
        
        shifterSolenoid = new DoubleSolenoid(0, 1);
        
        shooterMotor = new CANTalon(21);
        LiveWindow.addActuator("Shooter", "shooterMotor", shooterMotor);
        
        shooterPiston = new Solenoid(2, 3);
        LiveWindow.addActuator("Shooter", "shooterPiston", shooterPiston);
        
    }
}
