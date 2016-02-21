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
    	//addSequential(new IntakeLower());
    	addSequential(new DriveDistance(0.5, 500));			//Goes over ramparts
    	addSequential(new DriveAnglePID(90));				//Rotates towards the tower
    	addSequential(new DriveDistance(0.5, 500)); 		//Goes towards the tower
    	addSequential(new DriveStop());						//Stops moving? (This is possibly not needed)
    	//addSequential(new ShooterArmMoveToSetLocation(45)); //This will raise the shooter arm to prep the ball for shooting
    	//			Align robot using Vision to shoot the ball.
    	//addSequential(new ShootBall()); //Calls the ShootBall Command Group
    	//Then end Autonomous?  Is there anything else that needs to be done?
    	
    }
}
