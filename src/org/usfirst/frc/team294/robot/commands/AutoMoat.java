package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoMoat extends CommandGroup {
    
    public  AutoMoat() {
    	addSequential(new ShiftUp());
    	//addSequential(new DriveStraightSegInit(0.4, 40.0, DriveStraightSegInit.Units.inches));    	// "slow" speed, 40 inches
    	//addSequential(new DriveStraightSegMid(1.0, 12.0*12.0, DriveStraightSegMid.Units.inches));    	// "Fast" speed, 14 feet
    	addParallel(new ShooterArmMoveToSetLocation(RobotMap.upperBoundAngleToAvoid+3));
    	addSequential(new DriveStraightSegInit(.75, 10.0*12.0, DriveStraightSegInit.Units.inches));
    	addParallel(new IntakeLowerIfRaised());
    	addSequential(new DriveStraightSegMid(0.4, 12.0*12.0, DriveStraightSegMid.Units.inches));    	// "slow" speed, 1 feet
    	addSequential(new DriveStop());
    	//addSequential(new WaitSeconds(0.1));
//    	addSequential(new DriveAngle(0.55, 0, false, true));				// Turn towards goal using angle based on starting position knob
    	addSequential(new DriveAngle(0.7, 0.0, false));    		// Recover original orientation

    	addSequential(new ShiftDown());
    	addSequential(new WaitSeconds(1));
    	addSequential(new AutoDriveAndShootMiddleKnob());

//    	addParallel(new ShooterArmMoveToSetLocation(45));			// Start moving arm to correct target angle
//    	addSequential(new AutoTargetShoot());
    }
}
