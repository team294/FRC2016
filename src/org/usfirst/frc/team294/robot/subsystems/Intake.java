package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * Intake subsystem.  This needs a lot of work!
 */
public class Intake extends Subsystem {
	
    private final CANTalon intakeMotor = new CANTalon(RobotMap.intakeMotor);
        
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
    	
    }
    
    public void lowerIntake(){
    	
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

