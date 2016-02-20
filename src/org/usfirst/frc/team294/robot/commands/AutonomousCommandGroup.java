package org.usfirst.frc.team294.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousCommandGroup extends CommandGroup {
    
    public  AutonomousCommandGroup() {
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

    	addSequential(new DriveDistance(0.5, 1));
    	addSequential(new DriveAnglePID(90));
    	addSequential(new DriveDistance(0.5, 1)); 
    	addSequential(new DriveStop());
    	//addSequential(new RaiseShooterArm()); //This will raise the shooter arm to prep the ball for shooting
    	//Somewhere around here will need code to adjust for the goal to shoot correctly
    	//addSequential(new ShootBall()); //The robot will then shoot the ball, SCORING WITH 100% ACCURACY

    }
}
