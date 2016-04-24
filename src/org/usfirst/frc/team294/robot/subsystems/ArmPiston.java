package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ArmPiston extends Subsystem {

    private final Solenoid piston = new Solenoid(RobotMap.shifterSolenoidFwd, RobotMap.shifterSolenoidRev);
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	
	public void pistonOut(){
		piston.set(true);
	}
	
	public void pistonIn(){
		piston.set(false);
	}
	
	public boolean pistonIsOut(){
		return piston.get();
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}
