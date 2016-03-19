package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoPortcullis extends CommandGroup {
    
    public  AutoPortcullis() {
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
    	addSequential(new ShooterArmMoveToSetLocation(90));
//    	addSequential(new WaitSeconds(0.25));
    	addSequential(new WaitSeconds(0.1));
    	addSequential(new ShiftDown());
    	addSequential(new IntakeLowerIfRaised());
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.shooterArmBallCruiseAngle));
    	addSequential(new DriveStraightDistance(.6, 42, DriveStraightDistance.Units.inches));    	// "Slow" speed, 42 inches.  This should get to the base of the defense
    	addSequential(new IntakeSetToSpeed(-1));
    	addSequential(new DriveStraightDistance(.6, 10, DriveStraightDistance.Units.inches));    	// "Slow" speed, this will push up against the port
    	addSequential(new DriveStraightDistance(1.0, 6.0*12.0, DriveStraightDistance.Units.inches));    	// "Quickly blasts through port"
    	
    	
    	
    	//addSequential(new DriveAngle(0.55, 0, false));				// Recover original orientation
    }
}
