package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoDriveAndShootPosition3 extends CommandGroup {
    
    public  AutoDriveAndShootPosition3() {
    	addParallel(new ShooterArmMoveToSetLocation(45));			// Start moving arm to correct target angle
    	addSequential(new DriveStraightSegInit(1.0, 1.0*12.0, DriveStraightSegInit.Units.inches));
    	addSequential(new DriveStop());
//    	addSequential(new WaitSeconds(2.0));
    	addSequential(new DriveAngle(0.65, 15.0, true));    	
    	addSequential(new ShooterArmMoveToSetLocation(45));			// Finish moving arm to correct target angle
    	addSequential(new AutoTargetShoot());
    }
}
