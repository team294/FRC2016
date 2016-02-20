package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.OI;
import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.utilities.ToleranceChecker;

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
	private double minAngle=RobotMap.shooterArmMinAngle;
	private double maxAngle=RobotMap.shooterArmMaxAngle;
	private double anglesPerPos=(maxAngle-minAngle)/(maxPosition-minPosition);
	private double slope=(maxAngle/2-minAngle/2);
	private double yIntercept=maxAngle-slope;
	
	private ToleranceChecker tol = new ToleranceChecker(1.5, 5);

	public ShooterArm(){
		super(); 
		shooterArmMotor.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
		// shooterArmMotor.reverseSensor(true); 
		shooterArmMotor.setProfile(0);
		shooterArmMotor.setPID(0.020, 0.0000, 0.0);  
		shooterArmMotor.setF(0.0);   
		shooterArmMotor.changeControlMode(TalonControlMode.Position);
		shooterArmMotor.configPotentiometerTurns(1);
//		shooterArmMotor.set(shooterArmMotor.getAnalogInPosition());
//		shooterArmMotor.enableControl();
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
		if(Robot.intake.intakeIsUp()){
			if(Robot.shooterArm.getAngle() >=RobotMap.upperBoundAngleToAvoid&&angle<=RobotMap.upperBoundAngleToAvoid)
				angle=(RobotMap.upperBoundAngleToAvoid+3);
		}else if(Robot.shooterArm.getAngle()<=RobotMap.lowerBoundAngleToAvoid&&angle>=RobotMap.lowerBoundAngleToAvoid){
			angle=(RobotMap.lowerBoundAngleToAvoid-3);
		}
		if(angle>RobotMap.shooterArmMaxAngle){
			angle=RobotMap.shooterArmMaxAngle;
		}
		if(angle<RobotMap.shooterArmMinAngle){
			angle=RobotMap.shooterArmMinAngle;
		}
		shooterArmMotor.set(convertAngleToPos(angle));
		shooterArmMotor.enableControl();
		tol.reset();
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
		return tol.success( getAngle() - convertPosToAngle(shooterArmMotor.getSetpoint()) );  
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

