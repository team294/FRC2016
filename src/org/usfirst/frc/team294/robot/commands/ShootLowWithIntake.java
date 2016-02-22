package org.usfirst.frc.team294.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ShootLowWithIntake extends CommandGroup {
    
    public  ShootLowWithIntake() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.
    	addSequential(new FlyWheelSetToSpeed(5000)); //Starts the fly wheels, and runs them at full speed
    	addSequential(new IntakeSetToSpeed(-1));
    	addSequential(new ShooterPistonOut(true)); //When wheels are full speed, use piston to push the ball into fly wheels
    	addSequential(new WaitSeconds(0.5));
    	addSequential(new ShooterPistonOut(false));
    	addSequential(new FlyWheelSetToSpeed(0), 2); //Stops the fly wheels from spinning
    	addSequential(new IntakeSetToSpeed(0));
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
