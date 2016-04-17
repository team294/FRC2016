package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LowerIntakeAndShooterArm extends CommandGroup {
    
	/**
	 * Go from starting configuration (arm and intake up) to drive configuration (arm and intake down)
	 * @param cruiseAngle true = lower arm to cruise angle; false = lower arm to ball load angle (all the way)
	 */
    public LowerIntakeAndShooterArm(boolean cruiseAngle) {
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.upperBoundAngleToAvoid+3));
    	addSequential(new IntakeLowerIfRaised());
    	//addSequential(new WaitSeconds(0.5));
    	if (cruiseAngle)
    		addSequential(new ShooterArmMoveToSetLocation(RobotMap.shooterArmBallCruiseAngle));
    	else
    		addSequential(new ShooterArmMoveToSetLocation(RobotMap.shooterArmBallLoadAngle));    		
    }
}
