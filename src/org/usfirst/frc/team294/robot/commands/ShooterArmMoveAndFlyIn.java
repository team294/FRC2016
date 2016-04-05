package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ShooterArmMoveAndFlyIn extends CommandGroup {
    
    public ShooterArmMoveAndFlyIn(RobotMap.ShootFromLocation location) {
    	addParallel(new FlyWheelSetToSpeedIfArmIsLow(-500));
    	addParallel(new ShooterArmMoveToSetLocation(location));
    }
    
    public  ShooterArmMoveAndFlyIn(double angle) {
    	addParallel(new FlyWheelSetToSpeedIfArmIsLow(-500));
    	addParallel(new ShooterArmMoveToSetLocation(angle));
    }
}
