package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Autonomous code for going fast over low barriers (moat, rough terrain, rock wall, and ramparts).
 */
public class AutoCruiseConfig extends CommandGroup {
    
    public  AutoCruiseConfig() {
    	addSequential(new ShooterArmMoveToSetLocation(90));
//    	addSequential(new WaitSeconds(0.25));
    	addSequential(new WaitSeconds(0.1));
    	addSequential(new ShiftDown());
    	addSequential(new IntakeLowerIfRaised());
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.shooterArmBallCruiseAngle));
    }
}
