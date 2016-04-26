package org.usfirst.frc.team294.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ShooterArmMoveAndStopFlywheels extends CommandGroup {
    
    public  ShooterArmMoveAndStopFlywheels(double angle) {
    	addParallel(new FlyWheelStop());
    	addSequential(new ShooterArmMoveToSetLocation(angle));
    }

    public  ShooterArmMoveAndStopFlywheels(double angle, double tolerance) {
    	addParallel(new FlyWheelStop());
    	addSequential(new ShooterArmMoveToSetLocation(angle, tolerance));
    }
}
