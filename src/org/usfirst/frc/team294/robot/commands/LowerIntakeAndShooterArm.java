package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LowerIntakeAndShooterArm extends CommandGroup {
    
    public LowerIntakeAndShooterArm() {
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.upperBoundAngleToAvoid+3));
    	addSequential(new IntakeLowerIfRaised());
    	//addSequential(new WaitSeconds(0.5));
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.shooterArmBallCruiseAngle));
    }
}
