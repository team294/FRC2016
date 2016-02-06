package org.usfirst.frc294.RobotBuilderTest.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class shootingSequence extends CommandGroup {
    
    public  shootingSequence() {
    	
    	
    	addSequential(new flyWheels(true)); //Starts the fly wheels, and runs them at full speed
    	//addSequential(new flyWheelPiston(true)); //When wheels are full speed, use piston to push the ball into fly wheels
    	//addParallel(new flyWheelPiston(false));
    	addSequential(new flyWheels(false)); //Stops the fly wheels from spinning
    	
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
