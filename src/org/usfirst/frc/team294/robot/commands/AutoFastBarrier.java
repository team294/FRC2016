package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Autonomous code for going fast over low barriers (moat, rough terrain, rock wall, and ramparts).
 */
public class AutoFastBarrier extends CommandGroup {
    
    public  AutoFastBarrier() {
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.upperBoundAngleToAvoid+3));
    	//addSequential(new WaitSeconds(0.25));
    	addParallel(new ShiftDown());
    	addSequential(new IntakeLowerIfRaised());
    	addSequential(new WaitSeconds(0.1));
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.shooterArmBallCruiseAngle));
    	addSequential(new WaitSeconds(0.1));
    	addSequential(new DriveStraightDistance(0.75, 15.0*12.0, DriveStraightDistance.Units.inches));    	// "Fast" speed, 15 feet
    	addSequential(new WaitSeconds(0.1));
    	addParallel(new ShooterArmMoveToSetLocation(45));			// Start moving arm to correct target angle
    	addSequential(new WaitSeconds(0.1));
//    	addSequential(new WaitSeconds(0.2));
    	addSequential(new DriveAngle(0.55, 0, false, true));				// Turn towards goal using angle based on starting position knob
    	addSequential(new AutoTargetShoot());
    }
}
