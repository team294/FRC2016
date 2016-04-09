package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoCheval extends CommandGroup {
    
    public  AutoCheval() {
    	//
    	//Robot is 32 inches long
    	//
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.upperBoundAngleToAvoid+3));
//    	addSequential(new WaitSeconds(0.25));
//    	addSequential(new WaitSeconds(0.1));
    	addSequential(new ShiftDown());
    	addSequential(new DriveStraightDistance(.6, 45, DriveStraightDistance.Units.inches, 0.02));    	// "Slow" speed, 45 inches
    	addSequential(new IntakeLowerIfRaised());
    	addSequential(new WaitSeconds(0.1));
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.shooterArmBallCruiseAngle));
    	addSequential(new WaitSeconds(0.1));
    	addSequential(new DriveStraightDistance(.6, 15.0*12.0, DriveStraightDistance.Units.inches, 0.02));    	// "Slow" speed, 6 ft
    	addParallel(new ShooterArmMoveToSetLocation(45));			// Start moving arm to correct target angle
    	addSequential(new DriveAngle(0.55, 0, false, true));				// Turn towards goal using angle based on starting position knob
    	addSequential(new AutoTargetShoot());
    }
}
