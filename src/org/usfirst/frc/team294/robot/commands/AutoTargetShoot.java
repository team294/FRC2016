package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Automatically targets and shoots at the goal.
 */
public class AutoTargetShoot extends CommandGroup {
    
    public  AutoTargetShoot() {
    	// Target goal and pre-rev flywheels
    	addParallel(new FlyWheelSetToSpeedForGoal());		// Rev flywheels
//    	addParallel(new ShooterArmMoveToGoal());			// Start moving arm
    	addSequential(new DriveTurnToGoal(2.0),2);
    	addSequential(new DriveTurnToGoal(1.5),2);
//    	addSequential(new DriveTurnToGoal(1.5),2);			// Usually, it takes 2-3 turning iterations to get the position exactly.  But 3 iterations take time.
    	addSequential(new FlyWheelSetToSpeedForGoal()); 	// Ensure flywheels are at speed
    	// Move arm last, since it may move the target out of the camera's Y field of view!
    	addSequential(new ShooterArmMoveToGoal());			// Ensure arm is at target angle

    	// Shoot goal
    	addSequential(new ShooterPistonOut(true)); //When wheels are full speed, use piston to push the ball into fly wheels
    	addSequential(new WaitSeconds(0.5));
    	addSequential(new ShooterPistonOut(false));
    	addParallel(new FlyWheelStop()); //Stops the fly wheels from spinning
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.shooterArmBallCruiseAngle));
    	    	
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
