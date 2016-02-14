package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.*;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This is the robot drivetrain!
 */
public class DriveTrain extends Subsystem {
	
    private final CANTalon leftMotor1 = new CANTalon(RobotMap.driveTrainLeftMotor1);
    private final CANTalon leftMotor2 = new CANTalon(RobotMap.driveTrainLeftMotor2);
    private final CANTalon rightMotor1 = new CANTalon(RobotMap.driveTrainRightMotor1);
    private final CANTalon rightMotor2 = new CANTalon(RobotMap.driveTrainRightMotor2);
    private final RobotDrive robotDrive = new RobotDrive(leftMotor1, leftMotor2,
            rightMotor1, rightMotor2);
    private final AnalogGyro gyro = new AnalogGyro(RobotMap.driveTrainGyro1);

    public DriveTrain() {
    	// Call the Subsystem constructor
    	super();
    	
    	// Set up subsystem components
        leftMotor2.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        rightMotor2.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);

        robotDrive.setSafetyEnabled(true);
        robotDrive.setExpiration(0.1);
        robotDrive.setSensitivity(0.5);
        robotDrive.setMaxOutput(1.0);
    	
        gyro.setSensitivity(0.007);

        // Add the subsystem to the LiveWindow
        LiveWindow.addActuator("DriveTrain", "leftMotor1", leftMotor1);
        LiveWindow.addActuator("DriveTrain", "leftMotor2", leftMotor2);
        LiveWindow.addActuator("DriveTrain", "rightMotor1", rightMotor1);
        LiveWindow.addActuator("DriveTrain", "rightMotor2", rightMotor2);
        LiveWindow.addSensor("DriveTrain", "gyro1", gyro);
    }
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void driveWithJoystick(Joystick leftStick, Joystick rightStick) {
    	robotDrive.tankDrive(leftStick, rightStick);
    }

	public void stop(){
		robotDrive.drive(0, 0);
	}

	public void driveForward(double speed){
		robotDrive.drive(speed, 0);
	}

	public void driveBackward(double speed){
		robotDrive.drive(-1.0*speed, 0);
	}
	
	/**
	 * Drive the motors at "outputMagnitude" and "curve". 
	 * Both outputMagnitude and curve are -1.0 to +1.0 values, where 0.0 represents stopped and not turning. 
	 * curve < 0 will turn left and curve > 0 will turn right. The algorithm for steering provides a constant
	 * turn radius for any normal speed range, both forward and backward. Increasing m_sensitivity causes 
	 * sharper turns for fixed values of curve. This function will most likely be used in an autonomous 
	 * routine.
	 * @param speed The speed setting for the outside wheel in a turn, forward or backwards, +1 to -1.
	 * @param curve The rate of turn, constant for different forward speeds. Set curve < 0 for left turn or 
	 *          curve > 0 for right turn. Set curve = e^(-r/w) to get a turn radius r for wheelbase w of 
	 *          your robot. Conversely, turn radius r = -ln(curve)*w for a given value of curve and wheelbase w
	 */
	public void driveCurve(double speed, double curve){
		robotDrive.drive(speed, curve);
	}
	
	public double getDegrees(){
		SmartDashboard.putNumber("gyro angle", gyro.getAngle());
		return gyro.getAngle();
	}
	
	public void resetDegrees(){
		gyro.reset();
	}

	public int getLeftEncoder() {
		SmartDashboard.putNumber("Left Encoder Position", leftMotor2.getEncPosition());
		return leftMotor2.getEncPosition();
	}
	public int getRightEncoder(){
		SmartDashboard.putNumber("Right Encoder Position", rightMotor2.getEncPosition());
		return rightMotor2.getEncPosition();
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new DriveWithJoysticks());
   }
    
}

