package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Autonomous code for low bar.
 */
public class AutoLowBar extends CommandGroup {
    
    public  AutoLowBar() {
    	// Configure robot for low bar
    	addParallel(new LowerIntakeAndShooterArm(true));
    	addSequential(new ShiftDown());
    	addSequential(new WaitSeconds(2.5));	// Is this the right delay?

    	// Drive across barrier (and then some)
//    	addSequential(new DriveStraightDistance(0.65, 16.0*12.0, DriveStraightDistance.Units.inches));    	// "Slow" speed, 16 feet
    	addSequential(new DriveStraightSegInit(0.4, 40.0, DriveStraightSegInit.Units.inches));    	// "slow" speed, 40 inches
    	addSequential(new DriveStraightSegMid(0.65, 9.0*12.0, DriveStraightSegMid.Units.inches));    	// "Medium" speed, 9 feet total
    	addSequential(new DriveStraightSegMid(1.0, 15.0*12.0, DriveStraightSegMid.Units.inches));    	// "slow" speed, 15 feet total
    	addSequential(new DriveStraightSegMid(0.4, 16.0*12.0, DriveStraightSegMid.Units.inches));    	// "slow" speed, 16 feet total
    	addSequential(new DriveStop());
    	
    	// Turn towards goal
    	addParallel(new ShooterArmMoveToSetLocation(56));			// Start moving arm to correct target angle
    	addSequential(new WaitSeconds(0.1));
    	addSequential(new DriveAngle(0.55, 45, false));				// 45 degrees from original orientation
    	addSequential(new AutoTargetShoot());
   }
}