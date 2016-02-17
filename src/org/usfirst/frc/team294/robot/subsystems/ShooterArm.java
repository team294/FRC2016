package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ShooterArm extends Subsystem implements PIDOutput{
     private final CANTalon shooterArmMotor= new CANTalon(RobotMap.shooterArmMotor);
     private final AnalogInput shooterArmPot= new AnalogInput(RobotMap.shooterArmPot);
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

  // PID contorller and parameters for turning using the potentiometer, stolen from Don's PID code in the drivetrain
     PIDController angleController;
     static final double kP = 0.02;
     static final double kI = 0.00;
     static final double kD = 0.01;
     static final double kF = 0.00;
     static final double kToleranceDegrees = 3.0f;
     static final int kToleranceSamples = 5;  // These number of samples must be within tolerance to finish turn
     int nInToleranceSamples;  // Number of successive measurements that were in tolerance
     
     public ShooterArm(){
    	super(); 
    	//shooterArmMotor.setFeedbackDevice(shooterArmPot);
    	 
     }
     public double returnAngle(){
    	  return shooterArmPot.getAverageVoltage();
     }
     
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		
	}
}

