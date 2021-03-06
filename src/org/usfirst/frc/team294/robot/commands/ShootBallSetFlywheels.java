package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ShootBallSetFlywheels extends CommandGroup {
    
    public  ShootBallSetFlywheels() {		
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

    	addSequential(new IntakeSetToSpeedIfArmIsLow(-1.0));  // Turn on intake if we are shooting low
    	
//    	addSequential(new FlyWheelSetToSpeed(RobotMap.maxFlywheelSpeed)); //Starts the fly wheels, and runs them at full speed
    	addSequential(new FlyWheelSetToSpeed(2100, 2520)); //This is the speed for the poop shot
    	
    	addSequential(new ShooterPistonOut(true)); //When wheels are full speed, use piston to push the ball into fly wheels
    	addSequential(new WaitSeconds(.5));
    	addSequential(new ShooterPistonOut(false));
    	addSequential(new FlyWheelStop()); //Stops the fly wheels from spinning
    	addSequential(new IntakeSetToSpeed(0));  // Turn off intake motors
    }
}
