package org.usfirst.frc.team294.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Autonomous code for going fast over low barriers (moat, rough terrain, rock wall, and ramparts).
 */
public class AutoFastBarrier extends CommandGroup {
    
    public  AutoFastBarrier() {
    	addParallel(new LowerIntakeAndShooterArm());
    	addSequential(new ShiftDown());
    	addSequential(new DriveStraightSegInit(.4, 40.0, DriveStraightSegInit.Units.inches));    	// "slow" speed, 40 inches
    	addSequential(new DriveStraightSegFinal(1.0, 15.0*12.0, DriveStraightSegFinal.Units.inches));    	// "Fast" speed, 15 feet
    	//addSequential(new WaitSeconds(0.1));
    	addParallel(new ShooterArmMoveToSetLocation(45));			// Start moving arm to correct target angle
    	addSequential(new DriveAngle(0.55, 0, false, true));				// Turn towards goal using angle based on starting position knob
    	addSequential(new AutoTargetShoot());
    }
}
