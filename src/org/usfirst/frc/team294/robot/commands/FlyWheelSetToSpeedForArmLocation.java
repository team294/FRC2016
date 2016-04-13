package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ShootFromLocation;

public class FlyWheelSetToSpeedForArmLocation extends FlyWheelSetToSpeed {
    public FlyWheelSetToSpeedForArmLocation() {
    	super(RobotMap.ShootFromLocation.Batter);
	}
    
	// Called just before this Command runs the first time
    protected void initialize() {
    	ShootFromLocation location = Robot.shooterArm.getShootFromLocation();
        topSpeed = RobotMap.getTopSpeed(location);
        bottomSpeed = RobotMap.getBottomSpeed(location);
        super.initialize();
    }
}
