package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The shooter arm!
 */
public class Shooter extends Subsystem {

    private final CANTalon motorTop = new CANTalon(RobotMap.shooterMotorTop);
    private final CANTalon motorBottom = new CANTalon(RobotMap.shooterMotorBottom);
    private final DoubleSolenoid ballPiston = new DoubleSolenoid(RobotMap.shooterPistonFwd, RobotMap.shooterPistonRev);
    private final DigitalInput ballSenseButton = new DigitalInput(RobotMap.ballSenseButton);

    public Shooter() {
    	// Call the Subsystem constructor
    	super();
    	
    	// Set up subsystem components
        motorTop.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        motorTop.configEncoderCodesPerRev(100);
//        shooterMotorTop.reverseSensor(true);
        motorTop.configNominalOutputVoltage(+0.0f, -0.0f);
        motorTop.configPeakOutputVoltage(+12.0f, -12.0f);
        motorTop.setProfile(0);
        motorTop.setPID(0.020, 0.0002, 2.0);  // ProtoBot:  0.020, 0.0002, 2.0;  ProtoBoard:  0.005, 0.00008, 0.00001
        motorTop.setF(0.035);   // ProtoBot:  0.035;  ProtoBoard:  0.025
        motorTop.changeControlMode(TalonControlMode.Speed);
		
        motorBottom.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        motorBottom.configEncoderCodesPerRev(100);
//        shooterMotorBottom.reverseSensor(true);
        motorBottom.configNominalOutputVoltage(+0.0f, -0.0f);
        motorBottom.configPeakOutputVoltage(+12.0f, -12.0f);
        motorBottom.setProfile(0);
        motorTop.setPID(0.020, 0.0002, 2.0);  // ProtoBot:  0.020, 0.0002, 2.0;  ProtoBoard:  0.005, 0.00008, 0.00001
        motorTop.setF(0.035);   // ProtoBot:  0.035;  ProtoBoard:  0.025
        motorBottom.changeControlMode(TalonControlMode.Speed);
                
        ballPiston.set(DoubleSolenoid.Value.kReverse);
  
    	// Add the subsystem to the LiveWindow
        LiveWindow.addActuator("Shooter", "shooterMotorTop", motorTop);
        LiveWindow.addActuator("Shooter", "shooterMotorBottom", motorBottom);
        LiveWindow.addActuator("Shooter", "shooterPiston", ballPiston);
        LiveWindow.addSensor("Shooter", "ballSenseButton", ballSenseButton);
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
	/**
	 * Set shooter motor speeds.  Positive speed ejects the ball.  Negative speed loads the ball.  
	 */
    public void setSpeed(double speed){
    	motorTop.set(-speed*0.8);
    	motorBottom.set(speed);
    	SmartDashboard.putNumber("ShootTop Setpoint", motorTop.getSetpoint());
		SmartDashboard.putNumber("ShootBot Setpoint", motorBottom.getSetpoint());   	
    }
    
    public double getTopFlyWheelSpeed(){
    	return motorTop.getSpeed();
    }
    
    public double getBottomFlyWheelSpeed(){
    	return motorBottom.getSpeed();
    }
    
    public double getTopError(){
    	return motorTop.getError();
    }
    
    public double getBottomError(){
    	return motorBottom.getError();
    }
    
    public void setShooterPistonOut() {
    	ballPiston.set(DoubleSolenoid.Value.kForward);
    }

    public void setShooterPistonIn() {
    	ballPiston.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isButtonPressed(){
    	return !ballSenseButton.get();
    }

	/**
	 * Send shooter motor setpoint, speed, and error to SmartDashboard
	 */
    public void updateSmartDashboard() {
		SmartDashboard.putNumber("ShootTop Speed", motorTop.getSpeed());
		SmartDashboard.putNumber("ShootTop Error", motorTop.getError());
		
		SmartDashboard.putNumber("ShootBot Speed", motorBottom.getSpeed());
		SmartDashboard.putNumber("ShootBot Error", motorBottom.getError());		
    }
    
	/**
	 * Set up the shooter motor controls on the SmartDashboard.  Call this once when the robot is 
	 * initialized (after the Shooter subsystem is initialized).
	 * @param bPIDF false to only show setpoint/speed; true to also show the PIDF parameters 
	 */
    public void setupSmartDashboard(boolean bPIDF){
    	// bPID = TRUE to show PID parameters
    	
 		SmartDashboard.putNumber("ShootTop Setpoint", motorTop.getSetpoint());
		SmartDashboard.putNumber("ShootBot Setpoint", motorBottom.getSetpoint());
		updateSmartDashboard();

		if (bPIDF) {
			SmartDashboard.putNumber("ShootTop 1000*F", motorTop.getF()*1000);
			SmartDashboard.putNumber("ShootTop 1000*P", motorTop.getP()*1000);
			SmartDashboard.putNumber("ShootTop 1000*I", motorTop.getI()*1000);
			SmartDashboard.putNumber("ShootTop 1000*D", motorTop.getD()*1000);

			SmartDashboard.putNumber("ShootBot 1000*F", motorBottom.getF()*1000);
			SmartDashboard.putNumber("ShootBot 1000*P", motorBottom.getP()*1000);
			SmartDashboard.putNumber("ShootBot 1000*I", motorBottom.getI()*1000);
			SmartDashboard.putNumber("ShootBot 1000*D", motorBottom.getD()*1000);
		}
    }
    
	/**
	 * Set shooter motor setpoint and PIDF parameters from the SmartDashboard.  To use this method,
	 * be sure to previously call shooter.setupSmartDashboard(true) during robot init.
	 */
    public void setPIDFromSmartDashboard() {
		motorTop.set(SmartDashboard.getNumber("ShootTop Setpoint"));
		motorTop.setF(SmartDashboard.getNumber("ShootTop 1000*F")/1000);
		motorTop.setP(SmartDashboard.getNumber("ShootTop 1000*P")/1000);
		motorTop.setI(SmartDashboard.getNumber("ShootTop 1000*I")/1000);
		motorTop.setD(SmartDashboard.getNumber("ShootTop 1000*D")/1000);
		
		motorBottom.set(SmartDashboard.getNumber("ShootBot Setpoint"));
		motorBottom.setF(SmartDashboard.getNumber("ShootBot 1000*F")/1000);
		motorBottom.setP(SmartDashboard.getNumber("ShootBot 1000*P")/1000);
		motorBottom.setI(SmartDashboard.getNumber("ShootBot 1000*I")/1000);
		motorBottom.setD(SmartDashboard.getNumber("ShootBot 1000*D")/1000);
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}

