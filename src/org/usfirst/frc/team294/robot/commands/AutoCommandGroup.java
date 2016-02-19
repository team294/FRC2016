package org.usfirst.frc.team294.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoCommandGroup extends CommandGroup {
    
    public  AutoCommandGroup() {
    	
    	//addSequential(new DriveDistance()); //This will drive the robot a certain distance (Over obsticles too?)
    	//addSequential(new RotateDegreesPID()); //This will rotate the robot a certain amount of degrees to angle towards the goal.
    	//addSequential(new DriveDistance()); //This will make the robot drive towards the goal
    	//addSequential(new RaiseShooterArm()); //This will raise the shooter arm to prep the ball for shooting
    	//Somewhere around here will need code to adjust for the goal to shoot correctly
    	//addSequential(new ShootBall()); //The robot will then shoot the ball, SCORING WITH 100% ACCURACY
    	
    	
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
