package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.OI;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ShooterArm extends Subsystem {
	private final CANTalon shooterArmMotor= new CANTalon(RobotMap.shooterArmMotor);

	private double positionRange = 250.0;
	private double minPosition=233.0;		// We will need to calibrate this number occasionally
	private double maxPosition=minPosition + positionRange;
	private double minAngle=-12.0;
	private double maxAngle=94.0;
	private double anglesPerPos=(maxAngle-minAngle)/(maxPosition-minPosition);
	private double slope=(maxAngle/2-minAngle/2);
	private double yIntercept=41;


	public ShooterArm(){
		super(); 
		shooterArmMotor.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
		// shooterArmMotor.reverseSensor(true); 
		shooterArmMotor.setProfile(0);
		shooterArmMotor.setPID(0.020, 0.0000, 0.0);  
		shooterArmMotor.setF(0.0);   
		shooterArmMotor.changeControlMode(TalonControlMode.Position);
		shooterArmMotor.configPotentiometerTurns(1);
		shooterArmMotor.setPosition(shooterArmMotor.getAnalogInPosition());
		shooterArmMotor.enableControl();
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	/**
	 * Disable PID control of shooter arm
	 */
	public void disableControl() {
		shooterArmMotor.disableControl();
	}

	/**
	 * Get current arm angle
	 * @return angle, in degrees.  0 = horizontal, + = up, - = down
	 */
	public double getAngle() {
		return convertPosToAngle(shooterArmMotor.getAnalogInPosition());
	}

	/**
	 * Tell PID controller to move arm to a specific absolute angle.  Arm will move
	 * as much as it can within its movement limits and without interfering
	 * with the intake (if the intake is raised).
	 * @param angle Desired target angle, in degrees.  0 = horizontal, + = up, - = down
	 */
	public void moveToAngle(double angle) {
		// NEED TO FIX:  stay within limits and avoid intake
		if(RobotMap.intakeDown){
			shooterArmMotor.setPosition(convertAngleToPos(angle));
			shooterArmMotor.enableControl();
		} else if(!RobotMap.intakeDown){
			if(RobotMap.AngleOfShooterArm >=RobotMap.upperBoundAngleToAvoid)
				angle=(RobotMap.upperBoundAngleToAvoid+2);
			shooterArmMotor.setPosition(convertAngleToPos(angle));
			shooterArmMotor.enableControl();
		}
		if(RobotMap.AngleOfShooterArm<=RobotMap.lowerBoundAngleToAvoid){
			angle=(RobotMap.lowerBoundAngleToAvoid-2);
			shooterArmMotor.setPosition(convertAngleToPos(angle));
			shooterArmMotor.enableControl();	
		}
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
	 * Convert an arm angle to a PID position
	 * @param angle, in degrees
	 * @return PID position, 0 to 1023
	 */
	private double convertAngleToPos(double angle) {
		return ((angle-minAngle+(anglesPerPos*minPosition))/anglesPerPos);
	}

	/**
	 * Covert a PID position to an arm angle 
	 * @param position PID setting, 0 to 1023
	 * @return angle, in degrees
	 */
	private double convertPosToAngle(double position) {
		return ((anglesPerPos*position)-(anglesPerPos*minPosition)+minAngle);
	}
	public void convertJoystickToPosition(double stickVal){
		//stickVal=thirdJoystick.getY();
	}

	public void moveArmWithJoystick(Joystick thirdJoystick){
		moveToAngle((slope*thirdJoystick.getY())+yIntercept);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new MySpecialCommand());
	}
}

