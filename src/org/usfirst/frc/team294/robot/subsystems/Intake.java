package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Intake subsystem
 */
public class Intake extends Subsystem {
    private final CANTalon intakeMotor = new CANTalon(RobotMap.intakeMotor);
    private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(RobotMap.intakeSolenoidFwd, RobotMap.intakeSolenoidRev);
   
        
    public Intake() {
    	// Call the Subsystem constructor
    	super();
    	
    	// Set up subsystem components
    	intakeMotor.setVoltageRampRate(50);
    	
    	// Add the subsystem to the LiveWindow
        LiveWindow.addActuator("Intake", "Intake Motor", intakeMotor);
        LiveWindow.addActuator("Intake", "Intake Solenoid", intakeSolenoid);
    }

    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    /**
     * Set speed of the intake rollers
     * @param speed Roller motor power, -1 (expel ball) to +1 (load ball).  0 = stopped
     */
    public void setSpeed(double speed) {
    	intakeMotor.set(-speed);
    }
    
    /**
     * Get speed of the intake rollers
     * @return Roller motor power, -1 (expel ball) to +1 (load ball).  0 = stopped
     */
    public double getSpeed() {
    	return -intakeMotor.get();
    }
    
    /**
     * Raise intake arm, if the shooter arm is not in the way.  If shooter arm is in
     * the way, then this is ignored.
     */
    public void raiseIntake() {
    	if (!shooterArmConflicts()) {
    		intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    	}
    }
    
    /**
     * Lower intake arm
     */
    public void lowerIntake() {
    	if (!shooterArmConflicts()) {
        	intakeSolenoid.set(DoubleSolenoid.Value.kForward);    		
    	}
    }

    /**
     * Turn off piston solenoid
     */
    public void stopPiston() {
    	intakeSolenoid.set(DoubleSolenoid.Value.kOff);
    }
    
    /**
     * Returns current solenoid setting for the intake arm.  <p>
     * <b>NOTE</b>: The intake could currently be <i>moving</i> to this position
     * and has not reached this position yet.
     * @return true = intake is up, false = intake is down
     */
    public boolean intakeIsUp() {
    	switch (intakeSolenoid.get()) {
    	case kForward:
    		SmartDashboard.putString("Intake position", "Forward");
    		break;
    	case kReverse:
    		SmartDashboard.putString("Intake position", "Reverse");
    		break;
    	case kOff:
    		SmartDashboard.putString("Intake position", "Off");
    		break;
    	}
    	SmartDashboard.putBoolean("IntakeIsUp", intakeSolenoid.get()==DoubleSolenoid.Value.kReverse);
    	return intakeSolenoid.get()==DoubleSolenoid.Value.kReverse;
    }
    
    /**
     * Checks to make sure that the shooter arm is not going to crash into the intake when raising the intake
     * @return true when the shooter conflicts with the intake
     * @return false when the shooter does not conflict with the intake
     */
    public boolean shooterArmConflicts() {
    	double angleOfShooterArm = Robot.shooterArm.getAngle();
    	if (angleOfShooterArm >= RobotMap.lowerBoundAngleToAvoid && angleOfShooterArm <= RobotMap.upperBoundAngleToAvoid){
    		return true; 
    	}
    	return false; 
    }

	/**
	 * Set up the intake controls on the SmartDashboard.  Call this once when the robot is 
	 * initialized (after the Intake subsystem is initialized).
	 */
    public void setupSmartDashboard(boolean bPIDF){
		updateSmartDashboard();
    }
 
	/**
	 * Send intake status to SmartDashboard
	 */
    public void updateSmartDashboard() {
 		SmartDashboard.putNumber("Intake motor setpoint", -intakeMotor.get());
 		SmartDashboard.putString("Intake position", intakeIsUp() ? "Up" : "Down");
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

