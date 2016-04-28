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
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.upperBoundAngleToAvoid+8, 6));
//    	addSequential(new WaitSeconds(0.25));
//    	addSequential(new WaitSeconds(0.1));
    	addSequential(new ShiftDown());
    	addSequential(new DriveStraightDistance(.8, 45, DriveStraightDistance.Units.inches));    	// "Slow" speed, 45 inches
    	addSequential(new IntakeLowerIfRaised());
    	addSequential(new WaitSeconds(0.1));
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.shooterArmBallCruiseAngle, RobotMap.shooterArmBallCruiseAngleTolerance));
    	addSequential(new WaitSeconds(0.1));
//    	addSequential(new DriveStraightDistance(.6, 15.0*12.0, DriveStraightDistance.Units.inches));    	// "Slow" speed
//    	addSequential(new DriveStraightDistance(1.0, 8.0*12.0, DriveStraightDistance.Units.inches));    
    	addSequential(new DriveStraightSegInit(1.0, 6.0*12.0, DriveStraightSegInit.Units.inches));    // Reduced by 1 foot!
    	addSequential(new DriveStraightSegMid(0.6, 7.0*12.0, DriveStraightSegMid.Units.inches));
    	addSequential(new DriveStop());
//    	addSequential(new DriveAngle(0.55, 0, true));				// Turn towards goal using angle based on starting position knob

    	addSequential(new AutoDriveAndShootMiddleKnob());
    	
//    	addSequential(new DriveAngle(0.55, 0, false, true));				// Turn towards goal using angle based on starting position knob
//    	addSequential(new AutoTargetShoot());
    }
}
