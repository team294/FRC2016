package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoCheval extends CommandGroup {
    
    public  AutoCheval() {
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
    	//
    	//Robot is 32 inches long
    	//
    	addSequential(new ShooterArmMoveToSetLocation(90));
//    	addSequential(new WaitSeconds(0.25));
    	addSequential(new WaitSeconds(0.1));
    	addSequential(new ShiftDown());
    	addSequential(new DriveStraightDistance(.6, 50, DriveStraightDistance.Units.inches));    	// "Slow" speed, 45 inches
    	addSequential(new IntakeLowerIfRaised());
    	addSequential(new WaitSeconds(0.1));
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.shooterArmBallCruiseAngle));
    	addSequential(new WaitSeconds(0.1));
    	addSequential(new DriveStraightDistance(.6, 10.0*12.0, DriveStraightDistance.Units.inches));    	// "Slow" speed, 6 ft
    	addSequential(new DriveAngle(0.55, 0, false));				// Recover original orientation
    }
}
