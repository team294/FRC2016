package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Autonomous code for low bar.
 */
public class AutoLowBar extends CommandGroup {
    
    public  AutoLowBar() {
    	// Configure robot for low bar
    	addSequential(new ShooterArmMoveToSetLocation(90));
    	//addSequential(new WaitSeconds(0.25));
    	addParallel(new ShiftDown());
    	addSequential(new IntakeLowerIfRaised());
    	addSequential(new ShooterArmMoveToSetLocation(0));

    	// Drive across barrier and turn towards goal
    	addSequential(new DriveStraightDistance(0.65, 16.0*12.0, DriveStraightDistance.Units.inches));    	// "Slow" speed, 16 feet
    	
//    	addSequential(new DriveStraightSegInit());
//    	addSequential(new DriveStraightSegMid());
//    	addSequential(new DriveStraightSegMid());
//    	addSequential(new DriveStraightSegFinal());
    	
    	addParallel(new ShooterArmMoveToSetLocation(56));			// Start moving arm to correct target angle
    	addSequential(new DriveAngle(0.55, 45, false));				// 45 degrees from original orientation
    	/* don't target yet
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
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.shooterArmBallCruiseAngle));
    	    	*/
    	// Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}