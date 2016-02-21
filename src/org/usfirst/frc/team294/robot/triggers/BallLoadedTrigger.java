package org.usfirst.frc.team294.robot.triggers;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.buttons.Trigger;

public class BallLoadedTrigger extends Trigger {
	DigitalInput ballSensor;
	
	/** 
	 * Trigger to sense when a ball is loaded
	 * @param ballSensor Sensor to check
	 */
	public BallLoadedTrigger(DigitalInput ballSensor) {
		this.ballSensor = ballSensor;
	}
	
	/**
	 * Returns true when a ball is sensed
	 */
	public boolean get() {
		return !ballSensor.get();
	}
	
}
