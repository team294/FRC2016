package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Intake extends Subsystem {
	
	//These motors are the fly wheels, and need to spin slowly when intaking a ball
    private final CANTalon shooterMotorTop = RobotMap.shooterMotorTop;
    private final CANTalon shooterMotorBottom = RobotMap.shooterMotorBottom;
    //private final CANTalon intakeMotor = RobotMap.intakeMotor;
    
    private final DigitalInput intakeButton = RobotMap.intakeButton;
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public boolean isButtonPressed(){
    	return !intakeButton.get();
    }
    
    public void setSpeed(int speed){
//    	this.shooterMotorTop.set(speed);
//    	this.shooterMotorBottom.set(speed);
    }
    
    public void raiseIntake(){
    	
    }
    
    public void lowerIntake(){
    	
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

