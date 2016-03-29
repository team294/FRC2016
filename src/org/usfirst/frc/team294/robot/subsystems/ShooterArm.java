package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.OI;
import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.utilities.ToleranceChecker;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ShooterArm extends Subsystem {
	private final CANTalon shooterArmMotor= new CANTalon(RobotMap.shooterArmMotor);
    private final Solenoid brakeSolenoid = new Solenoid(RobotMap.shooterArmBrakeSolenoid);

	private double minPosition=Robot.armCalMinPosition;		// We will need to calibrate this number occasionally
	private double deg90Position=Robot.armCal90DegPosition;
//	private double minPosition=2.52;		// We will need to calibrate this number occasionally
//	private double maxPosition=2.27;		// The pot is "backwards" when the arm is fully down, the potentiometer is at a large value,
										// and when the pot is straight up, the pot is at a lower value.
	
	private double minAngle=RobotMap.shooterArmMinAngle;
	private double maxAngle=RobotMap.shooterArmMaxAngle;
	private double anglesPerPos=(90-minAngle)/(deg90Position-minPosition);
	
	private double joyAbsSlope=(maxAngle/2-minAngle/2);
	private double joyAbsYIntercept=maxAngle-joyAbsSlope;
	
	private double joyRelativeRate = 8;
	
	private ToleranceChecker armTol = new ToleranceChecker(1.5, 5);

	public ShooterArm(){
		super(); 
		
		shooterArmMotor.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
		//shooterArmMotor.reverseSensor(true); 
		//shooterArmMotor.setPID(12, 0.005, 0, 0, 20, 10000, 0);  // Good values without elastic retention bands, gears, 3-turn pot
		shooterArmMotor.setPID(43, 0.1, 0, 0, 20, 50, 0);  // Good values with elastic retention bands, gears, 3-turn pot (competition robot)
		shooterArmMotor.configPeakOutputVoltage(+8.0f, -10.0f);
		shooterArmMotor.changeControlMode(TalonControlMode.Position);
		shooterArmMotor.configPotentiometerTurns(3);		// 3-turn pot.  Also use this for 1-turn pot, since all cals are for this.
		shooterArmMotor.setReverseSoftLimit(deg90Position-0.06);		// Limit in high position (slightly more than 90 deg, 3-turn pot)
		shooterArmMotor.enableReverseSoftLimit(true);
		shooterArmMotor.setForwardSoftLimit(minPosition-0.02);		// Limit in low position (3-turn pot, slightly above 0 deg, and let gravity pull the arm down)
		shooterArmMotor.enableForwardSoftLimit(true);
		//shooterArmMotor.disableControl();
		
		setBrakeOff();
		
		setFlashlight(false);
		
    	// Add the subsystem to the LiveWindow
        LiveWindow.addActuator("ShooterArm", "shooterArmMotor", shooterArmMotor);
        LiveWindow.addActuator("ShooterArm", "shooterArmBrake",	brakeSolenoid);
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

    /**
     * Activate arm brake
     */
    public void setBrakeOn() {
    	SmartDashboard.putBoolean("Arm brake", true);
    	brakeSolenoid.set(true); 
    }

    /**
     * Deactivate arm brake
     */
    public void setBrakeOff() {
    	SmartDashboard.putBoolean("Arm brake", false);
    	brakeSolenoid.set(false);
    }

    /**
     * Reads the arm brake state
     * @return true = brake is on, false = brake is off
     */
    public boolean isBrakeOn() {
    	return brakeSolenoid.get();
    }
    
	/**
	 * Disable PID control of shooter arm
	 */
	public void disableControl() {
		shooterArmMotor.disableControl();
	}
	
	public double getPos(){
		return shooterArmMotor.get();
	}
	
	public double getEncPos(){
		return shooterArmMotor.getAnalogInPosition();
	}

	/**
	 * Get current arm angle
	 * @return angle, in degrees.  0 = horizontal, + = up, - = down
	 */
	public double getAngle() {
		return convertPosToAngle(shooterArmMotor.get());
	}

	/**
	 * Tell PID controller to move arm to a specific absolute angle.  Arm will move
	 * as much as it can within its movement limits and without interfering
	 * with the intake (if the intake is raised).
	 * Automatically turns flashlight on if angle >= 30 degrees, off if < 30 degrees.
	 * @param angle Desired target angle, in degrees.  0 = horizontal, + = up, - = down
	 */
	public void moveToAngle(double angle) {
//		if(angle > 35){
//    		SmartDashboard.putBoolean("useCamera1", false);
//    	}else{
//    		SmartDashboard.putBoolean("useCamera1", true);
//    	}
		
		
		// Don't move if the shooter arm is disabled or if arm could crash into intake
//		if (!Robot.shooterArmEnabled || 
//				(Robot.intake.intakeSolenoidIsOff() && getAngle()>=45 && angle<getAngle() )  ||
//				(Robot.intake.intakeSolenoidIsOff() && getAngle()<45 && angle>getAngle() )
//			) {
		if (!Robot.shooterArmEnabled) {
			SmartDashboard.putNumber("Arm set angle", -9999);			
			return;
		}
		
		// Turn off brake before moving arm
		setBrakeOff();
		
		// If arm is in keepout zone and intake is up (or in unknown state), then move arm away from the intake out of the keepout zone.
		if(Robot.intake.intakeIsUp() || Robot.intake.intakeSolenoidIsOff()){
//			if(getAngle()>=RobotMap.upperBoundAngleToAvoid && angle<=RobotMap.upperBoundAngleToAvoid){
//				angle=(RobotMap.upperBoundAngleToAvoid+3);
//			}else if(getAngle()<=RobotMap.lowerBoundAngleToAvoid && angle>=RobotMap.lowerBoundAngleToAvoid){
//				angle=(RobotMap.lowerBoundAngleToAvoid-3);
//			}
			if(getAngle()>=45 && angle<=RobotMap.upperBoundAngleToAvoid){
				angle=(RobotMap.upperBoundAngleToAvoid+3);
			}else if(getAngle()<45 && angle>=RobotMap.lowerBoundAngleToAvoid){
				angle=(RobotMap.lowerBoundAngleToAvoid-3);
			}
		}

		//Commenting this out will defeat the min/max arm safety
		if(angle>RobotMap.shooterArmMaxAngle){
			angle=RobotMap.shooterArmMaxAngle;
		}
		if(angle<RobotMap.shooterArmMinAngle){
			angle=RobotMap.shooterArmMinAngle;
		}
		
		SmartDashboard.putNumber("Arm set angle", angle);
        if (Robot.smartDashboardDebug) {
        	SmartDashboard.putNumber("Arm set position", convertAngleToPos(angle));
        }
        
		shooterArmMotor.clearIAccum();
		shooterArmMotor.set(convertAngleToPos(angle));
		shooterArmMotor.enableControl();
		armTol.reset();
		
		setFlashlight(angle>=30);
	}

	/**
	 * Tell PID controller to move arm up or down by a relative amount.  Arm will move
	 * as much as it can within its movement limits and without interfering
	 * with the intake (if the intake is raised).
	 * @param angle Desired relative movement, in degrees.  0 = none, + = higher, - = lower
	 */
	public void moveAngleRelative(double angle) {
		moveToAngle(getAngle() + angle);
	}

	/**
	 * Returns true if the arm angle has been within tolerance of its moveToAngle setpoint consistently
	 * while repeatedly calling this method.  Put this method in the calling command's isFinished() method.
	 * @return true = arm is at the setpoint
	 */
	public boolean moveToAngleIsFinished() {
//		return armTol.success( getAngle() - convertPosToAngle(shooterArmMotor.getSetpoint()) );  
		if (armTol.success( getAngle() - convertPosToAngle(shooterArmMotor.getSetpoint()) ) ) {
			setBrakeOn();
//			shooterArmMotor.set(convertAngleToPos(getAngle()));
//			shooterArmMotor.set(shooterArmMotor.get());
			return true;
		}
		return false;
	}
	
	/**
	 * Convert an arm angle to a PID position
	 * @param angle, in degrees
	 * @return PID position, 0 to 1023
	 */
	public double convertAngleToPos(double angle) {
		return ((angle-minAngle+(anglesPerPos*minPosition))/anglesPerPos);
	}

	/**
	 * Covert a PID position to an arm angle 
	 * @param position PID setting, 0 to 1023
	 * @return angle, in degrees
	 */
	public double convertPosToAngle(double position) {
		return ((anglesPerPos*position)-(anglesPerPos*minPosition)+minAngle);
	}
	
	public void convertJoystickToPosition(double stickVal){
		//stickVal=thirdJoystick.getY();
	}

	public void moveArmWithJoystickAbsolute(Joystick coJoystick){
		// Don't move if the shooter arm is disabled.
		if (!Robot.shooterArmEnabled) {
			SmartDashboard.putNumber("Arm set angle", -9999);			
			return;
		}

		moveToAngle((joyAbsSlope*coJoystick.getY())+joyAbsYIntercept);		
	}
	
	public void moveArmWithJoystickRelative(Joystick coJoystick) {
		// Don't move if the shooter arm is disabled.
		if (!Robot.shooterArmEnabled) {
			SmartDashboard.putNumber("Arm set angle", -9999);			
			return;
		}

//		SmartDashboard.putNumber("Arm Joystick Y", coJoystick.getY());
		
		moveAngleRelative(-coJoystick.getY()*joyRelativeRate);
	}
	
    /**
     * Turns on/off the targeting flashlight.
     * @param turnOn
     */
	public void setFlashlight(boolean turnOn) {
    	//TODO:  Implement method		
	}
	
	/**
	 * Set up the shooter motor controls on the SmartDashboard.  Call this once when the robot is 
	 * initialized (after the Shooter subsystem is initialized).
	 * @param bPIDF false to only show setpoint/speed; true to also show the PIDF parameters 
	 */
    public void setupSmartDashboard(boolean bPIDF){
    	// bPID = TRUE to show PID parameters
    	
 		SmartDashboard.putNumber("Arm Setpoint", shooterArmMotor.getSetpoint());

		if (bPIDF) {
			SmartDashboard.putNumber("Shooter Arm F", shooterArmMotor.getF());
			SmartDashboard.putNumber("Shooter Arm P", shooterArmMotor.getP());
			SmartDashboard.putNumber("Shooter Arm I", shooterArmMotor.getI());
			SmartDashboard.putNumber("Shooter Arm D", shooterArmMotor.getD());
		}
    }
    
    /**
     * Updates ShooterArm parameters on SmartDashboard.
     */
    public void updateSmartDashboard() {

    	SmartDashboard.putNumber("Arm Angle", getAngle());

        if (Robot.smartDashboardDebug) {
	    	SmartDashboard.putNumber("Arm Position", getPos());
	//        SmartDashboard.putNumber("Enc Position", getEncPos());
	//        SmartDashboard.putNumber("Arm Angle2", getAngle());
			SmartDashboard.putNumber("Arm Setpoint", shooterArmMotor.getSetpoint());
			SmartDashboard.putNumber("Arm Error", shooterArmMotor.getError());
	//		SmartDashboard.putBoolean("Arm Talon Mode", shooterArmMotor.getControlMode()==TalonControlMode.Position);
			SmartDashboard.putNumber("Arm motor voltage", shooterArmMotor.getOutputVoltage());
			SmartDashboard.putNumber("Arm talon bus voltage", shooterArmMotor.getBusVoltage());
			
			//This should reposition the arm to whatever angle that the user inputs.
			//moveToAngle(SmartDashboard.getNumber("Set angle"));
        }
    }
    
	/**
	 * Set shooter motor setpoint and PIDF parameters from the SmartDashboard.  To use this method,
	 * be sure to previously call shooter.setupSmartDashboard(true) during robot init.
	 */
    public void setPIDFromSmartDashboard() {
		shooterArmMotor.setF(SmartDashboard.getNumber("Shooter Arm F"));
		shooterArmMotor.setP(SmartDashboard.getNumber("Shooter Arm P"));
		shooterArmMotor.setI(SmartDashboard.getNumber("Shooter Arm I"));
		shooterArmMotor.setD(SmartDashboard.getNumber("Shooter Arm D"));
		
    }

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new MySpecialCommand());
	}
}

