package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoDriveAndShootPosition4 extends CommandGroup {
    
    public  AutoDriveAndShootPosition4() {
    	addParallel(new ShooterArmMoveToSetLocation(RobotMap.shootingAngleTargetAcquire));			// Start moving arm to correct target angle
    	addSequential(new DriveStraightSegInit(1.0, 1.0*12.0, DriveStraightSegInit.Units.inches));
    	addSequential(new DriveStop());
//    	addSequential(new WaitSeconds(2.0));
    	addSequential(new DriveAngle(0.65, -11.0, true));  			// Changed from -4 to -11 due to under-rotate in competition  	
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.shootingAngleTargetAcquire, 6.0));			// Finish moving arm to correct target angle
    	addSequential(new AutoTargetShoot());
    }
}
