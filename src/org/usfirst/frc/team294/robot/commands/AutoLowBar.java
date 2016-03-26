package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Autonomous code for low bar.
 */
public class AutoLowBar extends CommandGroup {
    
    public  AutoLowBar() {
    	addSequential(new ShooterArmMoveToSetLocation(90));
    	//addSequential(new WaitSeconds(0.25));
    	addSequential(new WaitSeconds(0.1));
    	addSequential(new ShiftDown());
    	addSequential(new IntakeLowerIfRaised());
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.shooterArmBallLoadAngle));
    	addSequential(new DriveStraightDistance(0.6, 15.0*12.0, DriveStraightDistance.Units.inches));    	// "Slow" speed, 15 feet
    	addSequential(new DriveAngle(0.55, 45, false));				// 45 degrees from original orientation
    	addSequential(new DriveStraightDistance(0.6, 2.0*12.0, DriveStraightDistance.Units.inches));    	// "Slow" speed, 2 feet
    	addSequential(new DriveAngle(0.55, 0, false));				// Straighten up    	
    }
}
