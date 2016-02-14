package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * This is the gear shifter for the main robot drive train.
 */
public class Shifter extends Subsystem {
    
    private final DoubleSolenoid shifter = new DoubleSolenoid(RobotMap.shifterSolenoidFwd, RobotMap.shifterSolenoidRev);
    
    public Shifter() {
    	// Call the Subsystem constructor
    	super();
    	
    	// Set up subsystem components

        // Add the subsystem to the LiveWindow
        LiveWindow.addActuator("Shifter", "shifter", shifter);
    }
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	public void shiftDown(){
		shifter.set(DoubleSolenoid.Value.kForward);
	}
	
	public void shiftUp(){
		shifter.set(DoubleSolenoid.Value.kReverse);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
}

