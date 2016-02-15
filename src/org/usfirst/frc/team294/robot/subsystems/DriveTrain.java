package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
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
    private final RobotDrive robotDrive = new RobotDrive(leftMotor1, leftMotor2,
            rightMotor1, rightMotor2);
    private final AnalogGyro gyro = new AnalogGyro(RobotMap.driveTrainGyro1);
    private AHRS ahrs;  // navX-mxp 9-axis IMU
    
    // PID contorller and parameters for turning using the navX
    PIDController turnController;
    static final double kP = 0.02;
    static final double kI = 0.00;
    static final double kD = 0.01;
    static final double kF = 0.00;
    static final double kToleranceDegrees = 3.0f;
    static final int kToleranceSamples = 5;  // These number of samples must be within tolerance to finish turn
    
    int nInToleranceSamples;  // Number of successive measurements that were in tolerance

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
        turnController.setInputRange(-180.0f,  180.0f);
        turnController.setContinuous(true);
        turnController.setOutputRange(-1.0, 1.0);
        turnController.setAbsoluteTolerance(kToleranceDegrees);  // PIDController.onTarget() method does not work!
        turnController.setToleranceBuffer(3);
        
        // Add the subsystem to the LiveWindow
        LiveWindow.addActuator("DriveTrain", "leftMotor1", leftMotor1);
        LiveWindow.addActuator("DriveTrain", "leftMotor2", leftMotor2);
        LiveWindow.addActuator("DriveTrain", "rightMotor1", rightMotor1);
        LiveWindow.addActuator("DriveTrain", "rightMotor2", rightMotor2);
        LiveWindow.addActuator("DriveTrain", "RotatePIDController", turnController);
        LiveWindow.addSensor("DriveTrain", "gyro1", gyro);
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

    public void driveWithJoystick(Joystick leftStick, Joystick rightStick) {
    	robotDrive.tankDrive(leftStick, rightStick);
    }

    /**
     * Stop the drive train motors
     */
	public void stop() {
		robotDrive.drive(0, 0);
	}

	/**
	 * Drive the robot straight forward
	 * @param speed +1 to -1, + = forward, - = backward
	 */
	public void driveForward(double speed) {
		robotDrive.drive(speed, 0);
	}

	/**
	 * Drive the robot straight backward
	 * @param speed +1 to -1, + = backward, - = forward
	 */
	public void driveBackward(double speed) {
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
		robotDrive.drive(speed, curve);
	}
	
	public double getDegrees() {
		SmartDashboard.putNumber("gyro angle", gyro.getAngle());
		SmartDashboard.putNumber("navX angle", ahrs.getAngle());
		return gyro.getAngle();
	}
	
	public void resetDegrees() {
		gyro.reset();
		ahrs.reset();
	}

	public void turnDegreesPIDStart(double degrees) {
		resetDegrees();
		nInToleranceSamples = 0;
		turnController.setSetpoint(degrees);
		turnController.enable();
	}
	
	public boolean turnDegreesPIDIsFinished() {
		double err;
		
//		SmartDashboard.putNumber("PID error", turnController.getError());
//		SmartDashboard.putNumber("PID avg error", turnController.getAvgError());
//		SmartDashboard.putNumber("PID setpoint", turnController.getSetpoint());
//		SmartDashboard.putNumber("PID power", turnController.get());
//		SmartDashboard.putBoolean("PID on target", turnController.onTarget());
	
		err = Math.abs(turnController.getSetpoint() - ahrs.getAngle());
		if (err > 180) { 
			err = Math.abs(360-err); 
		}
		
//		SmartDashboard.putNumber("PID my error", err);
		
		if ( err <= kToleranceDegrees ) {
			nInToleranceSamples++;
		} else {
			nInToleranceSamples = 0;
		}

		if ( nInToleranceSamples >= kToleranceSamples ) {
			turnController.disable();
			return true;
		} else {
			return false;
		}
}
	
	public void turnDegreesPIDCancel() {
		turnController.disable();
	}

	@Override
    /* This function is invoked periodically by the PID Controller, */
    /* based upon navX MXP yaw angle input and PID Coefficients.    */
    public void pidWrite(double output) {
//    	getDegrees();
//    	SmartDashboard.putNumber("PID turn power", output);
    	robotDrive.drive(output, -1);
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

