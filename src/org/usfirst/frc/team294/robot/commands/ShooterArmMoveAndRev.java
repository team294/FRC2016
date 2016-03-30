package org.usfirst.frc.team294.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ShooterArmMoveAndRev extends CommandGroup {
    
    public  ShooterArmMoveAndRev(double angle, double speedTop, double speedBottom) {
    	addParallel(new FlyWheelSetToSpeedIfArmIsLow(-500));
    	addParallel(new ShooterArmMoveToSetLocation(angle));
    	addSequential(new WaitSeconds(0.6));
    	addSequential(new FlyWheelSetToSpeed(speedTop, speedBottom));
    }
}
