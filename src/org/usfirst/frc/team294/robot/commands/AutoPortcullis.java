package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoPortcullis extends CommandGroup {
    
    public  AutoPortcullis() {
    	addSequential(new ShiftDown());
    	addParallel(new DriveStraightDistance(0.4, -2.0*12, DriveStraightDistance.Units.inches));    	// "slow" speed, 2 feet total
    	//addSequential(new LowerIntakeAndShooterArm(false));
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.upperBoundAngleToAvoid+3));
    	addSequential(new IntakeLowerIfRaised());
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.maxPistonOutAngle-3));
    	addSequential(new ArmPistonOut());
    	addSequential(new DriveStraightDistance(0.4, -2.0*12, DriveStraightDistance.Units.inches));    	// "slow" speed, 2 feet total
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.shooterArmBallLoadAngle));
    	addSequential(new DriveStraightDistance(1.0, -8.0*12.0, DriveStraightDistance.Units.inches));    	// "Fast" speed, 12 feet total
    	addSequential(new DriveStraightDistance(0.4, -1.0*12.0, DriveStraightDistance.Units.inches));    	// "slow" speed, 13 feet total
    	addParallel(new ArmPistonIn());
    	addSequential(new DriveStop());
    	//addSequential(new WaitSeconds(0.1));
//    	addSequential(new DriveAngle(0.55, 0, false, true));				// Turn towards goal using angle based on starting position knob
    	addSequential(new DriveAngle(0.7, 180.0, false));    		// Recover original orientation
    	
    	addSequential(new AutoDriveAndShootMiddleKnob());

//    	addParallel(new ShooterArmMoveToSetLocation(45));			// Start moving arm to correct target angle
//    	addSequential(new AutoTargetShoot());
    }
}
