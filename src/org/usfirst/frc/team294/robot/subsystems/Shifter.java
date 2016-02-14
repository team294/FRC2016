package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Shifter extends Subsystem {
    
    private final DoubleSolenoid shifter = RobotMap.shifterSolenoid;
	
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

