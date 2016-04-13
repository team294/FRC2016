package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ShooterArmMoveAndRev extends CommandGroup {
    
    public ShooterArmMoveAndRev(RobotMap.ShootFromLocation location) {
    	addParallel(new FlyWheelSetToSpeedIfArmIsLow(-500));
    	addParallel(new ShooterArmMoveToSetLocation(location));
    	addSequential(new WaitSeconds(0.3));			// Add short delay, so arm starts moving before next wait command (otherwise, next wait may read old arm setpoint) 
    	addSequential(new WaitForArmNearSetpoint(8.0)); // Wait for arm to get close to setpoint, so arm movement does not throw the ball into the flywheels while spinning up
    	addSequential(new FlyWheelSetToSpeed(location));
    }
    
    public  ShooterArmMoveAndRev(double angle, double speedTop, double speedBottom) {
    	addParallel(new FlyWheelSetToSpeedIfArmIsLow(-500));
    	addParallel(new ShooterArmMoveToSetLocation(angle));
    	addSequential(new WaitSeconds(0.3));			// Add short delay, so arm starts moving before next wait command (otherwise, next wait may read old arm setpoint) 
    	addSequential(new WaitForArmNearSetpoint(8.0)); // Wait for arm to get close to setpoint, so arm movement does not throw the ball into the flywheels while spinning up
    	addSequential(new FlyWheelSetToSpeed(speedTop, speedBottom));
    }
}
