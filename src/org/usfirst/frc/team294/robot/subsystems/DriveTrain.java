package org.usfirst.frc.team294.robot.subsystems;


import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.*;
import org.usfirst.frc.team294.robot.utilities.ToleranceChecker;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This is the robot drivetrain!
 */
public class DriveTrain extends Subsystem implements PIDOutput {
	
	// DriveTrain hardware
    private final CANTalon leftMotor1 = new CANTalon(RobotMap.driveTrainLeftMotor1);
    private final CANTalon leftMotor2 = new CANTalon(RobotMap.driveTrainLeftMotor2);
    private final CANTalon rightMotor1 = new CANTalon(RobotMap.driveTrainRightMotor1);
    private final CANTalon rightMotor2 = new CANTalon(RobotMap.driveTrainRightMotor2);
    private final RobotDrive robotDrive = new RobotDrive(leftMotor2, rightMotor2);
    private AHRS ahrs;  // navX-mxp 9-axis IMU
    
    // PID contorller and parameters for turning using the navX
    PIDController turnController;
    static final double kP = 0.04;
    static final double kI = 0.0;   //0.00025;
    static final double kD = 0.01;  //0.01;
    static final double kF = 0.00;
    static final double kToleranceDegrees = 1.5f;
    ToleranceChecker rotateTol = new ToleranceChecker(kToleranceDegrees, 5);
//    static final int kToleranceSamples = 5;  // These number of samples must be within tolerance to finish turn

    ToleranceChecker driveTol = new ToleranceChecker(100, 5);
    
//    static final double kDriveTolerance = 100.0f;
//    int nDriveInToleranceSamples;
    
//    int nInToleranceSamples;  // Number of successive measurements that were in tolerance

    public DriveTrain() {
    	// Call the Subsystem constructor
    	super();
    	
    	// Set up subsystem components
        leftMotor2.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        rightMotor2.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);

        setDriveControlByPower();
        
//      shooterMotorTop.reverseSensor(true);
        rightMotor2.setProfile(0);      
//      shooterMotorTop.reverseSensor(true);
        leftMotor2.setProfile(0);
      
        rightMotor2.setPID(0.0, 0.0, 0.0);  // ProtoBot:  0.020, 0.0002, 2.0;  ProtoBoard:  0.005, 0.00008, 0.00001
        leftMotor2.setPID(0.0, 0.0, 0.0);  // ProtoBot:  0.020, 0.0002, 2.0;  ProtoBoard:  0.005, 0.00008, 0.00001
        rightMotor2.setF(0.0);   // ProtoBot:  0.035;  ProtoBoard:  0.025
        leftMotor2.setF(0.0);   // ProtoBot:  0.035;  ProtoBoard:  0.025
      
        leftMotor1.changeControlMode(TalonControlMode.Follower);
        rightMotor1.changeControlMode(TalonControlMode.Follower);
        leftMotor1.set(leftMotor2.getDeviceID());
        rightMotor1.set(rightMotor2.getDeviceID());        
        
        robotDrive.setSafetyEnabled(true);
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
        
        // Implement PID controller for turning
        turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
        turnController.setInputRange(0.0f,  360.0f);
        turnController.setContinuous(true);
        turnController.setOutputRange(-1.0, 1.0);
        turnController.setAbsoluteTolerance(kToleranceDegrees);  // PIDController.onTarget() method does not work!
//        turnController.setToleranceBuffer(3);    
        
        
        // Add the subsystem to the LiveWindow
        LiveWindow.addActuator("DriveTrain", "leftMotor1", leftMotor1);
        LiveWindow.addActuator("DriveTrain", "leftMotor2", leftMotor2);
        LiveWindow.addActuator("DriveTrain", "rightMotor1", rightMotor1);
        LiveWindow.addActuator("DriveTrain", "rightMotor2", rightMotor2);
        LiveWindow.addActuator("DriveTrain", "RotatePIDController", turnController);
        LiveWindow.addSensor("DriveTrain", "navX-mxp", ahrs);
        
        /* Display 6-axis Processed Angle Data                                      */
//        SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
//        SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
//        SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
//        SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
//        SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
        
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
    }
    
    /**
     * Use this in the execute method of a DriveWithJoysticks command.  <p>
     * <b>NOTE:</b> Be sure to call setDriveControlByPower() in the initialize method.
     * @param leftStick Left joystick
     * @param rightStick Right joystick
     */
    public void driveWithJoystick(Joystick leftStick, Joystick rightStick) {
    	robotDrive.tankDrive(leftStick, rightStick);
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
		robotDrive.drive(speed, 0);
	}

	/**
	 * Drive the robot straight backward
	 * @param speed +1 to -1, + = backward, - = forward
	 */
	public void driveBackward(double speed) {
		setDriveControlByPower();
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
	public void driveCurve(double speed, double curve) {
		setDriveControlByPower();
		robotDrive.drive(speed, curve);
	}
	
	/**
	 * Returns the current angle of the gyro.
	 */
	public double getDegrees() {
		SmartDashboard.putNumber("navX angle", ahrs.getAngle());
		return ahrs.getAngle();
	}
	
	/**
	 * Resets the angle of the NavX
	 */
	public void resetDegrees() {
		ahrs.reset();
	}
	
	/**
	 * Turns the robot a certain number of degrees using PID.  Current PID
	 * parameters work only in low gear on test robot.
	 * @param degrees
	 */
	public void turnDegreesPIDStart(double degrees) {
		double d;
		
		if (degrees>=0) {
			d = degrees;
		} else {
			d = degrees + 360.0;
		}

		setDriveControlByPower();		
		
		resetDegrees();
//		nInToleranceSamples = 0;
		rotateTol.reset();
		turnController.reset();
		turnController.setSetpoint(d);
		turnController.enable();
	}
	
	@Override
    /* This function is invoked periodically by the rotation PID Controller, */
    /* based upon navX MXP yaw angle input and PID Coefficients.    */
    public void pidWrite(double output) {
//    	getDegrees();
//    	SmartDashboard.putNumber("PID turn power", output);
    	robotDrive.drive(output, -1);
    }
		
	/**
	 * Checks to see if the robot has turned a certain amount of degrees and is within the error range.
	 * @return boolean
	 */
	public boolean turnDegreesPIDIsFinished() {
		double err;
		
		SmartDashboard.putNumber("PID error", turnController.getError());
		SmartDashboard.putNumber("PID avg error", turnController.getAvgError());
		SmartDashboard.putNumber("PID setpoint", turnController.getSetpoint());
		SmartDashboard.putNumber("PID power", turnController.get());
//		SmartDashboard.putBoolean("PID on target", turnController.onTarget());
		SmartDashboard.putNumber("navX angle", ahrs.getAngle());
	
		err = Math.abs(turnController.getSetpoint() - ahrs.getAngle());
		if (err > 180) { 
			err = Math.abs(360-err); 
		}
		
//		SmartDashboard.putNumber("PID my error", err);
		
//		if ( err <= kToleranceDegrees ) {
//			nInToleranceSamples++;
//		} else {
//			nInToleranceSamples = 0;
//		}

//		if ( nInToleranceSamples >= kToleranceSamples ) {
		if ( rotateTol.success(err) ) {
			turnController.reset();
			rotateTol.reset();
			return true;
		} else {
			return false;
		}
	}
	
	/**
	 * Stops the PID Rotating.
	 */
	public void turnDegreesPIDCancel() {
		turnController.reset();
		rotateTol.reset();
	}

	/**
	 * Drives the robot forward a certain distance
	 * @param distance (eg. 5000 encoder units)
	 */
	public void driveDistancePIDStart(double distance){
		rightMotor2.changeControlMode(TalonControlMode.Position);
		leftMotor2.changeControlMode(TalonControlMode.Position);
        rightMotor2.setPID(0.020, 0.0002, 2.0);  // ProtoBot:  0.020, 0.0002, 2.0;  ProtoBoard:  0.005, 0.00008, 0.00001
        leftMotor2.setPID(0.020, 0.0002, 2.0);  // ProtoBot:  0.020, 0.0002, 2.0;  ProtoBoard:  0.005, 0.00008, 0.00001
        rightMotor2.setF(0.035);   // ProtoBot:  0.035;  ProtoBoard:  0.025
        leftMotor2.setF(0.035);   // ProtoBot:  0.035;  ProtoBoard:  0.025
		rightMotor2.set(distance);
		leftMotor2.set(distance);
		rightMotor2.enableControl();
		leftMotor2.enableControl();
	}
	
	/**
	 * Checks to see if the robot has reached the distance it was assigned to
	 * @return boolean
	 */
	public boolean driveDistancePIDIsFinished(){
		double errRight;
		double errLeft;

		//err = Math.abs(turnController.getSetpoint() - ahrs.getAngle());
		errRight = Math.abs(rightMotor2.getError());
		errLeft = Math.abs(leftMotor2.getError());
		
//		if ( errLeft <= kDriveTolerance && errRight <= kDriveTolerance) {
//			nDriveInToleranceSamples++;
//		} else {
//			nDriveInToleranceSamples = 0;
//		}
//
//		if ( nDriveInToleranceSamples >= kToleranceSamples ) {
		if ( driveTol.success(errLeft+errRight) ) {
			driveDistancePIDCancel();
			return true;
		} else {
			return false;
		}
	}
	
	/**
	 * Stops the PID Control of Driving the robot forward
	 */
	public void driveDistancePIDCancel(){
		rightMotor2.disableControl();
		leftMotor2.disableControl();
		setDriveControlByPower();
		rightMotor2.setPID(0.0, 0.0, 0.0);  // ProtoBot:  0.020, 0.0002, 2.0;  ProtoBoard:  0.005, 0.00008, 0.00001
        leftMotor2.setPID(0.0, 0.0, 0.0);  // ProtoBot:  0.020, 0.0002, 2.0;  ProtoBoard:  0.005, 0.00008, 0.00001
        rightMotor2.setF(0.0);   // ProtoBot:  0.035;  ProtoBoard:  0.025
        leftMotor2.setF(0.0);   // ProtoBot:  0.035;  ProtoBoard:  0.025
	}

	public int getLeftEncoder() {
		SmartDashboard.putNumber("Left Encoder Position", leftMotor2.getEncPosition());
		return leftMotor2.getEncPosition();
	}
	public int getRightEncoder() {
		SmartDashboard.putNumber("Right Encoder Position", rightMotor2.getEncPosition());
		return rightMotor2.getEncPosition();
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new DriveWithJoysticks());
   }
    
}

