package org.usfirst.frc.team294.robot.subsystems;


import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;


/**
 * This is the robot drivetrain!
 */
public class DriveTrain extends Subsystem {
	
	// DriveTrain hardware
    private final CANTalon leftMotor1 = new CANTalon(RobotMap.driveTrainLeftMotor1);
    private final CANTalon leftMotor2 = new CANTalon(RobotMap.driveTrainLeftMotor2);
    private final CANTalon rightMotor1 = new CANTalon(RobotMap.driveTrainRightMotor1);
    private final CANTalon rightMotor2 = new CANTalon(RobotMap.driveTrainRightMotor2);
    private final RobotDrive robotDrive = new RobotDrive(leftMotor2, rightMotor2);
    private AHRS ahrs;  // navX-mxp 9-axis IMU
    
    // Track encoder resets in software, since encoder reset on CANTalon has latency to the next encoder read
    private double leftEncoderZero = 0, rightEncoderZero = 0;
    
    // Track navX resets in software, since gyro reset on navX has latency to the next encoder read
    private double yawZero = 0;
    
    public DriveTrain() {
    	// Call the Subsystem constructor
    	super();
    	
    	// Set up subsystem components
        leftMotor2.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        rightMotor2.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        leftMotor2.configEncoderCodesPerRev(100);
        rightMotor2.configEncoderCodesPerRev(100);
        leftMotor2.configNominalOutputVoltage(+0.0f, -0.0f);
        rightMotor2.configNominalOutputVoltage(+0.0f, -0.0f);
        leftMotor2.configPeakOutputVoltage(+12.0f, -12.0f);
        rightMotor2.configPeakOutputVoltage(+12.0f, -12.0f);
        leftMotor2.setVoltageRampRate(40);
        rightMotor2.setVoltageRampRate(40);
        // With these settings, the encoder reads 4000 ticks per revolution.
        
        leftMotor2.reverseSensor(true);

        setDriveControlByPower();
        
//        rightMotor2.setProfile(0);      
//        leftMotor2.setProfile(0);      
//        rightMotor2.setPID(0.5, 0.0, 0.0);  // ProtoBot:  0.020, 0.0002, 2.0;  ProtoBoard:  0.005, 0.00008, 0.00001
//        leftMotor2.setPID(0.5, 0.0, 0.0);  // ProtoBot:  0.020, 0.0002, 2.0;  ProtoBoard:  0.005, 0.00008, 0.00001
//        rightMotor2.setF(0.0);   // ProtoBot:  0.035;  ProtoBoard:  0.025
//        leftMotor2.setF(0.0);   // ProtoBot:  0.035;  ProtoBoard:  0.025
      
        leftMotor1.changeControlMode(TalonControlMode.Follower);
        rightMotor1.changeControlMode(TalonControlMode.Follower);
        leftMotor1.set(leftMotor2.getDeviceID());
        rightMotor1.set(rightMotor2.getDeviceID());        
        
        robotDrive.setExpiration(0.1);
        robotDrive.setSensitivity(0.5);
        robotDrive.setMaxOutput(1.0);
    	
        try {
            /* Communicate w/navX MXP via the MXP SPI Bus.                                     */
            /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
            /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
            ahrs = new AHRS(SPI.Port.kMXP); 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
        ahrs.zeroYaw();        
        
        // Add the subsystem to the LiveWindow
        LiveWindow.addActuator("DriveTrain", "leftMotor1", leftMotor1);
        LiveWindow.addActuator("DriveTrain", "leftMotor2", leftMotor2);
        LiveWindow.addActuator("DriveTrain", "rightMotor1", rightMotor1);
        LiveWindow.addActuator("DriveTrain", "rightMotor2", rightMotor2);
        LiveWindow.addSensor("DriveTrain", "navX-mxp", ahrs);
        
        /* Display 6-axis Processed Angle Data                                      */
        SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
        SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
        SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
        SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
        SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
        
        /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
        /* path for upgrading from the Kit-of-Parts gyro to the navx MXP            */
//        SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
//        SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());
    }
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    /**
     * Set the drive controller to use power settings, instead of using the
     * encoder PID controller.
     */
    public void setDriveControlByPower() {
        leftMotor2.changeControlMode(TalonControlMode.PercentVbus);    	
        rightMotor2.changeControlMode(TalonControlMode.PercentVbus);
        leftMotor2.configPeakOutputVoltage(+12.0f, -12.0f);
        rightMotor2.configPeakOutputVoltage(+12.0f, -12.0f);
        robotDrive.setSafetyEnabled(true);
    }
    
    /**
     * Use this in the execute method of a DriveWithJoysticks command.  <p>
     * <b>NOTE:</b> Be sure to call setDriveControlByPower() in the initialize method.
     * @param leftStick Left joystick
     * @param rightStick Right joystick
     *
    public void driveWithJoystick(Joystick leftStick, Joystick rightStick) {
        leftMotor2.clearStickyFaults();
        rightMotor2.clearStickyFaults();
    	robotDrive.tankDrive(leftStick, rightStick);
    }*/
    
    /**
     * Use this in the execute method of a DriveWithJoysticks command.  <p>
     * <b>NOTE:</b> Be sure to call setDriveControlByPower() in the initialize method.
     * @param leftStick Left joystick
     * @param rightStick Right joystick
     */
    public void driveWithJoystick(double leftStick, double rightStick) {
    	robotDrive.tankDrive(leftStick, rightStick, false);
    }

    /**
     * Stop the drive train motors
     */
	public void stop() {
		setDriveControlByPower();
		robotDrive.drive(0, 0);
	}

	/**
	 * Drive the robot straight forward
	 * @param speed +1 to -1, + = forward, - = backward
	 */
	public void driveForward(double speed) {
		setDriveControlByPower();
		robotDrive.drive(-speed, 0);
	}

	/**
	 * Drive the robot straight backward
	 * @param speed +1 to -1, + = backward, - = forward
	 */
	public void driveBackward(double speed) {
		setDriveControlByPower();
		robotDrive.drive(speed, 0);
	}
	
	/**
	 * Drive the motors at "speed" and "curve". 
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
	public void driveCurve(double speed, double curve) {
		setDriveControlByPower();
		robotDrive.drive(-speed, curve);
	}
	
	/**
	 * Resets the angle of the NavX gyro.
	 */
	public void resetDegrees() {
//		ahrs.reset();
//		System.out.println("Just reset NavX.  Current angle = " + ahrs.getAngle());

	    // Track navX resets in software, since gyro reset on navX has latency to the next encoder read
		yawZero = ahrs.getAngle();
	}
	
	/**
	 * Returns the current angle of the NavX gyro.
	 */
	public double getDegrees() {
		double angle;
		
		angle = ahrs.getAngle() - yawZero; 
		
		// Normalize to 0 to 360 degrees
		angle = angle - Math.floor(angle/360)*360;
		
		SmartDashboard.putNumber("navX angle", angle>180.0 ? angle-360.0 : angle);
		return angle;
	}
	
	public void smartDashboardNavXAngles() {
		SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
		SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
		SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
	}
	
	/**
	 * Gets tilting forward/back angle (pitch)
	 * @return angle in degrees, 0 = vertical, + = nose down, - = nose up
	 */
	public double getRobotPitch() {
		return ahrs.getRoll();		// Note that NavX orientation is 90 degrees, so swap pitch/roll
	}
	
	/**
	 * Gets tilting left/right angle (roll)
	 * @return angle in degrees, 0 = vertical, + = tilted (left?), - = tilted (right?) -- need to check
	 */
	public double getRobotRoll() {
		return ahrs.getPitch();		// Note that NavX orientation is 90 degrees, so swap pitch/roll
	}
	
	/**
	 * Reset encoder positions to 0.
	 */
	public void resetEncoders() {
//        leftMotor2.setPosition(0);
//        rightMotor2.setPosition(0);

        // Track encoder resets in software, since encoder reset on CANTalon has latency to the next encoder read
        leftEncoderZero = leftMotor2.getPosition();
        rightEncoderZero = rightMotor2.getPosition();
	}
	
	/*
	 * Get left encoder position.  4000 ticks = 1 revolution.
	 */
	public double getLeftEncoder() {
		if (Robot.smartDashboardDebug) {
//			SmartDashboard.putNumber("Left Setpoint", leftMotor2.getSetpoint());
			SmartDashboard.putNumber("Left Position", leftMotor2.getPosition() - leftEncoderZero);
//			SmartDashboard.putNumber("Left Encoder Position", leftMotor2.getEncPosition());
//			SmartDashboard.putNumber("Left Get", leftMotor2.get());
//			SmartDashboard.putNumber("Left Error", leftMotor2.getError());
			SmartDashboard.putNumber("Left Output Voltage", leftMotor2.getOutputVoltage());
			SmartDashboard.putNumber("Left Speed", leftMotor2.getSpeed());
			
			SmartDashboard.putBoolean("Left Mode Position", TalonControlMode.Position == leftMotor2.getControlMode());
		}
		
		return leftMotor2.getPosition() - leftEncoderZero;
	}
	
	/*
	 * Get right encoder position.  4000 ticks = 1 revolution.
	 */
	public double getRightEncoder() {
		if (Robot.smartDashboardDebug) {
//			SmartDashboard.putNumber("Right Setpoint", rightMotor2.getSetpoint());
			SmartDashboard.putNumber("Right Position", rightMotor2.getPosition() - rightEncoderZero);
//			SmartDashboard.putNumber("Right Encoder Position", rightMotor2.getEncPosition());
//			SmartDashboard.putNumber("Right Get", rightMotor2.get());
//			SmartDashboard.putNumber("Right Error", rightMotor2.getError());
			SmartDashboard.putNumber("Right Output Voltage", rightMotor2.getOutputVoltage());
			SmartDashboard.putNumber("Right Speed", rightMotor2.getSpeed());
	
			SmartDashboard.putBoolean("Right Mode Position", TalonControlMode.Position == rightMotor2.getControlMode());
		}
		
		return rightMotor2.getPosition() - rightEncoderZero;
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new DriveWithJoysticks());
   }
    
}

