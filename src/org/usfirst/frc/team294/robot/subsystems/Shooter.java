package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ShootFromLocation;
import org.usfirst.frc.team294.robot.commands.FlyWheelStop;
import org.usfirst.frc.team294.robot.commands.RecordBallState;
import org.usfirst.frc.team294.robot.triggers.BallLoadedTrigger;
import org.usfirst.frc.team294.robot.triggers.MotorCurrentTrigger;
import org.usfirst.frc.team294.robot.utilities.RCSwitch;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DoubleSolenoid;  // Old code for double solenoid from prototype bot
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.hal.CanTalonJNI;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The shooter mechanism on the arm.
 */
public class Shooter extends Subsystem {

    private final CANTalon motorTop = new CANTalon(RobotMap.shooterMotorTop);
    private final CANTalon motorBottom = new CANTalon(RobotMap.shooterMotorBottom);
//    private final DoubleSolenoid ballPiston = new DoubleSolenoid(RobotMap.shooterPistonFwd, RobotMap.shooterPistonRev);  // Old code for double solenoid from prototype bot
    private final Solenoid ballPiston = new Solenoid(RobotMap.shooterPiston);
    private final DigitalInput ballSensor = new DigitalInput(RobotMap.ballSensor);

    private RCSwitch speedlight = new RCSwitch(RobotMap.speedlight);
    
	public boolean bLEDsFlywheelAtSpeed = false;
	public boolean bLEDsArmAtAngle = false;

	private final BallLoadedTrigger ballLoadedTrigger = new BallLoadedTrigger(ballSensor);
    
    boolean ballIsLoaded = false;

    private final MotorCurrentTrigger motorTopCurrentTrigger = new MotorCurrentTrigger(motorTop, 30, 4);
    private final MotorCurrentTrigger motorBottomCurrentTrigger = new MotorCurrentTrigger(motorBottom, 30, 4);

    /**
     * Create a shooter
     */
    public Shooter() {
    	// Call the Subsystem constructor
    	super();
    	
    	// Set up subsystem components
        motorTop.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        motorTop.configEncoderCodesPerRev(100);
        motorTop.reverseSensor(true);
        motorTop.configNominalOutputVoltage(+0.0f, -0.0f);
        motorTop.configPeakOutputVoltage(+12.0f, -12.0f);
//        motorTop.setPID(0.010, 0.00005, 0); motorTop.setF(0.020); // Better (3" wheels)
//        motorTop.setPID(0.010, 0.00015, 0, 0.020, 10000, 50, 0); // Limit windup -- best (3" wheels)
        motorTop.setPID(0.060, 0.00015, 0, 0.025, 6000, 50, 0); // best (4" wheels)
        motorTop.changeControlMode(TalonControlMode.Speed);

        motorBottom.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        motorBottom.configEncoderCodesPerRev(100);
        motorBottom.reverseSensor(true);
        motorBottom.configNominalOutputVoltage(+0.0f, -0.0f);
        motorBottom.configPeakOutputVoltage(+12.0f, -12.0f);
        motorBottom.setPID(0.060, 0.00015, 0, 0.030, 6000, 50, 0); // best (4" wheels)
        motorBottom.changeControlMode(TalonControlMode.Speed);
                
//        ballPiston.set(DoubleSolenoid.Value.kReverse);
        ballPiston.set(false);
        
        setFlywheelSpeedLight(false);
        
        ballLoadedTrigger.whenActive(new RecordBallState(true));
        
        // motor stall protection
        //motorTopCurrentTrigger.whenActive(new FlyWheelStop());
        //motorBottomCurrentTrigger.whenActive(new FlyWheelStop());
  
    	// Add the subsystem to the LiveWindow
        LiveWindow.addActuator("Shooter", "shooterMotorTop", motorTop);
        LiveWindow.addActuator("Shooter", "shooterMotorBottom", motorBottom);
        LiveWindow.addActuator("Shooter", "shooterPiston", ballPiston);
        LiveWindow.addSensor("Shooter", "ballSenseButton", ballSensor);
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
	/**
	 * Set shooter motor speeds.  WARNING:  Keep speed <= RobotMap.maxFlywheelSpeed RPM to ensure that the PIDF
	 * controller can achieve the desired speed.  If not, then the I term of the PIDF
	 * will prevent the controller from changing speeds properly.
     * @param speed RPM to set both top and bottom motors.  + = eject ball, - = load ball.
	 */
    public void setSpeed(double speed) {
    	setSpeed(speed, speed);
    }
    
	/**
	 * Set shooter motor speeds.  WARNING:  Keep speeds <= RobotMap.maxFlywheelSpeed RPM to ensure that the PIDF
	 * controller can achieve the desired speed.  If not, then the I term of the PIDF
	 * will prevent the controller from changing speeds properly.
	 * <p> This method allows separate top/bottom speeds in order to put a spin on the ball.
     * @param topSpeed RPM to set top motor.  + = eject ball, - = load ball.
     * @param bottomSpeed RPM to set bottom motor.  + = eject ball, - = load ball.
	 */
    public void setSpeed(double topSpeed, double bottomSpeed) {
    	
    	if(topSpeed > 0 && bottomSpeed > 0){
    		Robot.vision.setCameraPeriod(0);
    	}else{
    		Robot.vision.setCameraPeriod(1);
    	}
    	
    	motorTop.enableControl();
    	motorBottom.enableControl();
    	if (Math.abs(motorTop.getSetpoint()-topSpeed)>0.1*topSpeed) {
        	motorTop.clearIAccum();    		
    	}
    	if (Math.abs(motorBottom.getSetpoint()-bottomSpeed)>0.1*bottomSpeed) {
        	motorBottom.clearIAccum();    		
    	}
    	motorTop.set(topSpeed);
    	motorBottom.set(bottomSpeed);

		if (Robot.smartDashboardDebug) {
	    	SmartDashboard.putNumber("ShootTop Setpoint", motorTop.getSetpoint());
			SmartDashboard.putNumber("ShootBot Setpoint", motorBottom.getSetpoint());   	
		}
    }
    
    /**
     * Stops the flywheels using break mode of the talon.  Turns off the
     * flywheel speed light.
     */
    public void stopFlyWheels() {
    	motorTop.disableControl();
    	motorBottom.disableControl();
    	
    	setFlywheelSpeedLight(false);
		Robot.vision.setCameraPeriod(1);
    }
    
    /** 
     * Get speed of top flywheel motor
     * @return Speed in RPM
     */
    public double getTopFlyWheelSpeed(){
//    	SmartDashboard.putNumber("ShootTop SpeedTest", motorTop.getSpeed());

    	return motorTop.getSpeed();
    }
    
    /** 
     * Get speed of bottom flywheel motor
     * @return Speed in RPM
     */
   public double getBottomFlyWheelSpeed(){
//		SmartDashboard.putNumber("ShootBot SpeedTest", motorBottom.getSpeed());

		return motorBottom.getSpeed();
    }
    
   /** 
    * Get error for top flywheel motor = actual speed - set speed
    * @return Speed in RPM
    */   
    public double getTopError(){
    	return motorTop.getError();
    }
    
    /** 
     * Get error for bottom flywheel motor = actual speed - set speed
     * @return Speed in RPM
     */   
    public double getBottomError(){
    	return motorBottom.getError();
    }
    
    /**
     * Extend shooter piston
     */
    public void setShooterPistonOut() {
//    	ballPiston.set(DoubleSolenoid.Value.kForward);  // Old code for double solenoid from prototype bot
    	ballPiston.set(true); 
    	recordBallState(false);  // Record that we shot the ball
    }

    /**
     * Retract shooter piston
     */
    public void setShooterPistonIn() {
//    	ballPiston.set(DoubleSolenoid.Value.kReverse);  // Old code for double solenoid from prototype bot
    	ballPiston.set(false);
    	recordBallState(false);  // Record that we shot the ball
    }

    /**
     * Reads the shooter piston state
     * @return true = piston is out
     */
    public boolean getShooterPistonPosition() {
    	return ballPiston.get();
    }
    
    /**
     * Call this method to record the loading or unloading of a ball for state tracking by the shooter.
     * This is autmatically updated when a ball is loaded by the BallLoadedTrigger.
     * @param ballIsLoaded true = ball is loaded, false = ball is not loaded
     */
    public void recordBallState(boolean ballIsLoaded) {
    	this.ballIsLoaded = ballIsLoaded;
    }
    
    /**
     * Checks if the shooter has a ball
     * @return true = shooter has a ball
     */
    public boolean isBallLoaded(){
//    	SmartDashboard.putBoolean("BallSensor", ballSensor.get());
    	SmartDashboard.putBoolean("Ball in shooter", ballIsLoaded);
    	return ballIsLoaded;
    }

    /**
     * Turns on/off the light to show that the flywheels are spun up to speed.
     * @param turnOn
     */
    public void setFlywheelSpeedLight(boolean turnOn) {
    	speedlight.set(turnOn);
    }
    
    /**
     * Notify LEDs if flywheels are at set speed (forward direction only) and turn LEDs on/off as appropriate
     * @param atSpeed, true = flywheels are at speed
     */
    public void setLEDsFlywheelAtSpeed(boolean atSpeed) {
    	bLEDsFlywheelAtSpeed = atSpeed;
//    	setFlywheelSpeedLight(bLEDsFlywheelAtSpeed && bLEDsArmAtAngle);
    	// Code to turn LEDs on/off and flash are now in Robot.teleopPeriodic()
    }
    
    /**
     * Notify LEDs if arm is at set angle and turn LEDs on/off as appropriate
     * @param atAngle, true = arm is at speed
     */
    public void setLEDsArmAtAngle(boolean atAngle) {
    	bLEDsArmAtAngle = atAngle;
//    	setFlywheelSpeedLight(bLEDsFlywheelAtSpeed && bLEDsArmAtAngle);
    	// Code to turn LEDs on/off and flash are now in Robot.teleopPeriodic()
    }
    
	/**
	 * Send shooter motor setpoint, speed, and error to SmartDashboard
	 */
    public void updateSmartDashboard() {
		SmartDashboard.putNumber("ShootTop Speed", motorTop.getSpeed());
		SmartDashboard.putNumber("ShootTop Error", motorTop.getError());
		
		SmartDashboard.putNumber("ShootBot Speed", motorBottom.getSpeed());
		SmartDashboard.putNumber("ShootBot Error", motorBottom.getError());		

//		SmartDashboard.putNumber("ShootTop Speed2", motorTop.getSpeed());
//		SmartDashboard.putNumber("ShootBot Speed2", motorBottom.getSpeed());    
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

