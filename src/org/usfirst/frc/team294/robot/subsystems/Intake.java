package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

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

    	// Add the subsystem to the LiveWindow
        LiveWindow.addActuator("Intake", "intakeMotor", intakeMotor);
    }

    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setSpeed(int speed){
    	intakeMotor.set(speed);
    }
    
    public void raiseIntake(){
    	intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    
    public void lowerIntake(){
    	intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    
    /*
     * checks to make sure that the shooter arm is not going to crash into the intake when raising the intake
     * @returns true when the shooter conflicts with the intake
     * @returns false when the shooter does not conflict with the intake
     */
    public boolean shooterArmConflicts(){
    	double AngleOfShooterArm = Robot.shooterArm.getAngle();
    	if (AngleOfShooterArm > RobotMap.lowerBoundAngleToAvoid&&AngleOfShooterArm<RobotMap.upperBoundAngleToAvoid){
    		return true; 
    	}
    	return false; 
    }
    
    public void stop(){
    	intakeSolenoid.set(DoubleSolenoid.Value.kOff);
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

