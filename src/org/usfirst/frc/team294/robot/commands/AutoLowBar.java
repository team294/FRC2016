package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Autonomous code for low bar.
 */
public class AutoLowBar extends CommandGroup {
    
    public  AutoLowBar() {
    	// Configure robot for low bar
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.upperBoundAngleToAvoid+3));
    	//addSequential(new WaitSeconds(0.25));
    	addParallel(new ShiftDown());
    	addSequential(new IntakeLowerIfRaised());
    	addSequential(new WaitSeconds(0.1));
    	addSequential(new ShooterArmMoveToSetLocation(0));
    	addSequential(new WaitSeconds(0.1));

    	// Drive across barrier and turn towards goal
    	addSequential(new DriveStraightDistance(0.65, 16.0*12.0, DriveStraightDistance.Units.inches));    	// "Slow" speed, 16 feet
    	
//    	addSequential(new DriveStraightSegInit(1.0, 2.0*12.0, DriveStraightSegInit.Units.inches));
//    	addSequential(new DriveStraightSegMid(0.4, 8.0*12.0, DriveStraightSegMid.Units.inches));
//    	addSequential(new DriveStraightSegMid(1.0, 12.0*12.0, DriveStraightSegMid.Units.inches));
//    	addSequential(new DriveStraightSegFinal(0.4, 16.0*12.0, DriveStraightSegFinal.Units.inches));

    	addSequential(new WaitSeconds(0.1));
    	addParallel(new ShooterArmMoveToSetLocation(56));			// Start moving arm to correct target angle
    	addSequential(new WaitSeconds(0.1));
    	addSequential(new DriveAngle(0.55, 45, false));				// 45 degrees from original orientation
    	addSequential(new AutoTargetShoot());
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