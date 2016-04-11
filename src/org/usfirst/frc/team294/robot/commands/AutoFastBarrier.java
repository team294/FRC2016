package org.usfirst.frc.team294.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Autonomous code for going fast over low barriers (moat, rough terrain, rock wall, and ramparts).
 */
public class AutoFastBarrier extends CommandGroup {
    
    public  AutoFastBarrier() {
    	addParallel(new LowerIntakeAndShooterArm());
    	addSequential(new ShiftDown());
    	addSequential(new WaitSeconds(2.0));
    	addSequential(new DriveStraightSegInit(0.4, 40.0, DriveStraightSegInit.Units.inches));    	// "slow" speed, 40 inches
    	addSequential(new DriveStraightSegMid(1.0, 14.0*12.0, DriveStraightSegMid.Units.inches));    	// "Fast" speed, 14 feet
    	addSequential(new DriveStraightSegFinal(0.4, 15.0*12.0, DriveStraightSegFinal.Units.inches));    	// "slow" speed, 1 feet
    	//addSequential(new WaitSeconds(0.1));
    	addParallel(new ShooterArmMoveToSetLocation(45));			// Start moving arm to correct target angle
    	addSequential(new DriveAngle(0.55, 0, false, true));				// Turn towards goal using angle based on starting position knob
    	addSequential(new AutoTargetShoot());
    }
}
