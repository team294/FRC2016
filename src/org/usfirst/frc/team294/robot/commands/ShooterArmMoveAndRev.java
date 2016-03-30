package org.usfirst.frc.team294.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ShooterArmMoveAndRev extends CommandGroup {
    
    public  ShooterArmMoveAndRev(double angle, double speedTop, double speedBottom) {
    	addSequential(new FlyWheelSetToSpeedIfArmIsLow());
    	addSequential(new ShooterArmMoveToSetLocation(angle));
    	addSequential(new FlyWheelSetToSpeed(speedTop, speedBottom));
    }
}
