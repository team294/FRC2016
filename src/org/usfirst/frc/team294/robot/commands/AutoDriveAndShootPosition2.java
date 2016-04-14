package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoDriveAndShootPosition2 extends CommandGroup {
    
    public  AutoDriveAndShootPosition2() {
    	addParallel(new ShooterArmMoveToSetLocation(45));			// Start moving arm to correct target angle
    	addSequential(new DriveAngle(0.7, 60.0, false));
    	addSequential(new DriveStraightSegInit(1.0, 4.0*12.0, DriveStraightSegInit.Units.inches));
    	addSequential(new DriveStop());
//    	addSequential(new WaitSeconds(2.0));
    	addSequential(new DriveAngle(0.7, -40.0, false));    	
    	addSequential(new AutoTargetShoot());
    }
}
