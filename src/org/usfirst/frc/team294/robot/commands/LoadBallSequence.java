package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LoadBallSequence extends CommandGroup {
    
    public  LoadBallSequence() {
    	addSequential(new ShooterPistonOut(false));	// Bring piston in, just in case it is out.  Also resets ball sensor = "no ball loaded"
    	addParallel(new FlyWheelSetToSpeed(-3000));
    	addSequential(new ShooterArmMoveAwayFromIntake(ShooterArmMoveAwayFromIntake.condition.ifIntakeNotInWayAndIntakeIsUp));  // Also waits for move to finish
    	addSequential(new IntakeLowerIfRaised());  // Also waits for move to finish
    	addSequential(new ShooterArmMoveToSetLocation(RobotMap.shooterArmBallLoadAngle), 0.75);
    	
    	addParallel(new IntakeSetToSpeed(1));
//    	addParallel(new FlyWheelSetToSpeed(-3000));
    	addSequential(new WaitForBallLoaded());
    	
    	addSequential(new WaitSeconds(.5));
    	addParallel(new IntakeSetToSpeed(0)); 
    	addSequential(new FlyWheelStop());

    	addSequential(new WaitSeconds(.2));			// Wait for intake to stop before reversing direction
    	addSequential(new IntakeSetToSpeed(-1)); 

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
