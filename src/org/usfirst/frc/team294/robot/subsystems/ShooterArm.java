package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ShooterArm extends Subsystem {
	private final CANTalon shooterArmMotor= new CANTalon(RobotMap.shooterArmMotor);
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private static double maxPosition=433.0;
	private static double minPosition=233.0;
	private static double maxAngle=94.0;
	private static double minAngle=-12.0;
	private static double anglesPerPos=(maxAngle-minAngle)/(maxPosition-minPosition);

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
	public double getAngle(){
		return convertPosToAngle(shooterArmMotor.getAnalogInPosition());
	}

	public void moveToAngle(double angle){
		shooterArmMotor.setPosition(convertAngleToPos(angle));
	}
	
	public double convertAngleToPos(double angle){
		return ((angle-minAngle+(anglesPerPos*minPosition))/anglesPerPos);
	}
	public double convertPosToAngle(double position){
		return ((anglesPerPos*position)-(anglesPerPos*minPosition)+minAngle);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new MySpecialCommand());
	}
}

