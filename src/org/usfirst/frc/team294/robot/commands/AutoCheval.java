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
    	addSequential(new ShooterArmMoveToSetLocation(90));
//    	addSequential(new WaitSeconds(0.25));
    	addSequential(new WaitSeconds(0.1));
    	addSequential(new ShiftDown());
    	addSequential(new DriveStraightDistance(.6, 45, DriveStraightDistance.Units.inches));    	// "Slow" speed, 45 inches
    	addSequential(new IntakeLowerIfRaised());
    	addSequential(new WaitSeconds(0.1));
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.shooterArmBallCruiseAngle));
    	addSequential(new WaitSeconds(0.1));
    	addSequential(new DriveStraightDistance(.6, 10.0*12.0, DriveStraightDistance.Units.inches));    	// "Slow" speed, 6 ft
    	addSequential(new DriveAngle(0.55, 0, false));				// Recover original orientation
/*
    	// Target goal and pre-rev flywheels
    	addParallel(new FlyWheelSetToSpeedForGoal());		// Rev flywheels
    	addParallel(new ShooterArmMoveToGoal());			// Start moving arm
    	addSequential(new DriveTurnToGoal(2.0));
    	addSequential(new DriveTurnToGoal(1.0));
    	addSequential(new DriveTurnToGoal(1.0));
    	addSequential(new ShooterArmMoveToGoal());			// Ensure arm is at target angle

    	// Shoot goal
    	addSequential(new FlyWheelSetToSpeedForGoal()); 	// Ensure flywheels are at speed
    	addSequential(new ShooterPistonOut(true)); //When wheels are full speed, use piston to push the ball into fly wheels
    	addSequential(new WaitSeconds(0.5));
    	addSequential(new ShooterPistonOut(false));
    	addParallel(new FlyWheelStop()); //Stops the fly wheels from spinning
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.shooterArmBallCruiseAngle));*/
    }
}
