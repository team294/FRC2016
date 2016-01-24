
package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Motor1 extends Subsystem {
	CANTalon left1 = new CANTalon(RobotMap.Left1);
	CANTalon left2 = new CANTalon(RobotMap.Left2); 
	CANTalon right1 = new CANTalon(RobotMap.Right1);
	CANTalon right2 = new CANTalon(RobotMap.Right2);
	DoubleSolenoid doubleSolenoid = new DoubleSolenoid(RobotMap.SolenoidChannel1,RobotMap.SolenoidChannel2);
	Gyro gyro = new AnalogGyro(RobotMap.Gyro); 
	
	public Motor1(){
	right2.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
	left2.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
	}
	
	public void shiftDown(){
		doubleSolenoid.set(DoubleSolenoid.Value.kForward);
	}
	
	public void shiftUp(){
		doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
	}
	
	public void driveforward(double time, double speed){
		driveLeft(speed);
		driveRight(speed); 
	}
	public void driveLeft(double speed){
			left1.set(speed);
			left2.set(speed);
	}
	public void driveRight(double speed){
		right1.set(speed);
		right2.set(speed);

	}
	public void backwards(double speed){
		driveLeft(speed*-1);
		driveRight(speed*-1);
		
	}
	public double getDegrees(){
		SmartDashboard.putNumber("gyro angle", gyro.getAngle());
		return gyro.getAngle();
	}
	public void resetDegrees(){
		gyro.reset();
	}

	public int getLeftEncoder() {
		SmartDashboard.putNumber("Left Encoder Position", left2.getEncPosition());
		return left2.getEncPosition();
	}
	public int getRightEncoder(){
		SmartDashboard.putNumber("Right Encoder Position", right2.getEncPosition());
		return right2.getEncPosition();
	}
	public void stop(){
		
	}
	public void initDefaultCommand() {
	}
	// Set the default command for a subsystem here


}

