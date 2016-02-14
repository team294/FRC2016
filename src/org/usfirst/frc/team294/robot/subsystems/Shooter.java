package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.*;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 */
public class Shooter extends Subsystem {

    private final CANTalon shooterMotorTop = RobotMap.shooterMotorTop;
    private final CANTalon shooterMotorBottom = RobotMap.shooterMotorBottom;
    private final DoubleSolenoid shooterPiston = RobotMap.shooterPiston;
    private final DigitalInput intakeButton = RobotMap.intakeButton;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
	/**
	 * Set shooter motor speeds.  Positive speed ejects the ball.  Negative speed loads the ball.  
	 */
    public void setSpeed(double speed){
    	shooterMotorTop.set(-speed*0.8);
    	shooterMotorBottom.set(speed);
    	SmartDashboard.putNumber("ShootTop Setpoint", shooterMotorTop.getSetpoint());
		SmartDashboard.putNumber("ShootBot Setpoint", shooterMotorBottom.getSetpoint());   	
    }
    
    public double getTopFlyWheelSpeed(){
    	return shooterMotorTop.getSpeed();
    }
    
    public double getBottomFlyWheelSpeed(){
    	return shooterMotorBottom.getSpeed();
    }
    
    public double getTopError(){
    	return shooterMotorTop.getError();
    }
    
    public double getBottomError(){
    	return shooterMotorBottom.getError();
    }
    
    public void setShooterPistonOut() {
    	shooterPiston.set(DoubleSolenoid.Value.kForward);
    }

    public void setShooterPistonIn() {
    	shooterPiston.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isButtonPressed(){
    	return !intakeButton.get();
    }

	/**
	 * Send shooter motor setpoint, speed, and error to SmartDashboard
	 */
    public void updateSmartDashboard() {
		SmartDashboard.putNumber("ShootTop Speed", shooterMotorTop.getSpeed());
		SmartDashboard.putNumber("ShootTop Error", shooterMotorTop.getError());
		
		SmartDashboard.putNumber("ShootBot Speed", shooterMotorBottom.getSpeed());
		SmartDashboard.putNumber("ShootBot Error", shooterMotorBottom.getError());		
    }
    
	/**
	 * Set up the shooter motor controls on the SmartDashboard.  Call this once when the robot is 
	 * initialized (after the Shooter subsystem is initialized).
	 * @param bPIDF false to only show setpoint/speed; true to also show the PIDF parameters 
	 */
    public void setupSmartDashboard(boolean bPIDF){
    	// bPID = TRUE to show PID parameters
    	
 		SmartDashboard.putNumber("ShootTop Setpoint", shooterMotorTop.getSetpoint());
		SmartDashboard.putNumber("ShootBot Setpoint", shooterMotorBottom.getSetpoint());
		updateSmartDashboard();

		if (bPIDF) {
			SmartDashboard.putNumber("ShootTop 1000*F", shooterMotorTop.getF()*1000);
			SmartDashboard.putNumber("ShootTop 1000*P", shooterMotorTop.getP()*1000);
			SmartDashboard.putNumber("ShootTop 1000*I", shooterMotorTop.getI()*1000);
			SmartDashboard.putNumber("ShootTop 1000*D", shooterMotorTop.getD()*1000);

			SmartDashboard.putNumber("ShootBot 1000*F", shooterMotorBottom.getF()*1000);
			SmartDashboard.putNumber("ShootBot 1000*P", shooterMotorBottom.getP()*1000);
			SmartDashboard.putNumber("ShootBot 1000*I", shooterMotorBottom.getI()*1000);
			SmartDashboard.putNumber("ShootBot 1000*D", shooterMotorBottom.getD()*1000);
		}
    }
    
	/**
	 * Set shooter motor setpoint and PIDF parameters from the SmartDashboard.  To use this method,
	 * be sure to previously call shooter.setupSmartDashboard(true) during robot init.
	 */
    public void setPIDFromSmartDashboard() {
		shooterMotorTop.set(SmartDashboard.getNumber("ShootTop Setpoint"));
		shooterMotorTop.setF(SmartDashboard.getNumber("ShootTop 1000*F")/1000);
		shooterMotorTop.setP(SmartDashboard.getNumber("ShootTop 1000*P")/1000);
		shooterMotorTop.setI(SmartDashboard.getNumber("ShootTop 1000*I")/1000);
		shooterMotorTop.setD(SmartDashboard.getNumber("ShootTop 1000*D")/1000);
		
		shooterMotorBottom.set(SmartDashboard.getNumber("ShootBot Setpoint"));
		shooterMotorBottom.setF(SmartDashboard.getNumber("ShootBot 1000*F")/1000);
		shooterMotorBottom.setP(SmartDashboard.getNumber("ShootBot 1000*P")/1000);
		shooterMotorBottom.setI(SmartDashboard.getNumber("ShootBot 1000*I")/1000);
		shooterMotorBottom.setD(SmartDashboard.getNumber("ShootBot 1000*D")/1000);
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}

