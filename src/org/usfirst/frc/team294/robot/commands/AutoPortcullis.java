package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoPortcullis extends CommandGroup {
    
    public  AutoPortcullis() {
    	addSequential(new ShooterArmMoveToSetLocation(90));
//    	addSequential(new WaitSeconds(0.25));
    	addSequential(new WaitSeconds(0.1));
    	addSequential(new ShiftDown());
    	addSequential(new IntakeLowerIfRaised());
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.shooterArmBallLoadAngle));
    	addSequential(new DriveStraightDistance(.5, 42, DriveStraightDistance.Units.inches));    	// "Slow" speed, 42 inches.  This should get to the base of the defense
    	addSequential(new IntakeSetToSpeed(-1));
    	addSequential(new DriveStraightDistance(.5, 24, DriveStraightDistance.Units.inches));    	// "Slow" speed, this will push up against the port
    	addSequential(new DriveStraightDistance(1.0, 6.0*12.0, DriveStraightDistance.Units.inches));    	// "Quickly blasts through port"
    	addSequential(new DriveAngle(0.55, 0, false));				// Recover original orientation
    }
}
