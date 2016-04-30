package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoDriveAndShootPosition2a extends CommandGroup {
    
	/**
	 * Position 2, but go straight (far) and turn.  No dog leg.
	 */
    public  AutoDriveAndShootPosition2a() {
    	addParallel(new ShooterArmMoveToSetLocation(RobotMap.shootingAngleTargetAcquire));			// Start moving arm to correct target angle
//    	addSequential(new DriveAngle(0.7, 60.0, false));
    	addSequential(new DriveStraightSegInit(1.0, 4.0*12.0, DriveStraightSegInit.Units.inches));
    	addSequential(new DriveStop());
    	//addSequential(new WaitSeconds(2.0));
    	//addSequential(new DriveAngle(0.7, -30, false));    	//This could work for other defenses
    	addSequential(new DriveAngle(0.7, 45, false));    	//Testing for possible portcullis auto.
    	addSequential(new AutoTargetShoot());
    }
}
