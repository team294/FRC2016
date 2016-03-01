package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.control.Conditional;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LoadBallSequence extends CommandGroup {
    
    public  LoadBallSequence() {
    	
    	addSequential(new Conditional(new IntakeLower(), null) {
    		protected boolean condition() {
    			return Robot.intake.intakeIsUp();
    		}
    		});
    	
//    	if(Robot.intake.intakeIsUp()){
//    		addSequential(new IntakeLower());
//    		addSequential(new WaitSeconds(2));
//    	} 

    	// NEED TO UNCOMMENT AND TEST THIS WHEN THE ARM CODE IS READY
    	
//    	if(Robot.shooterArm.getAngle()>RobotMap.shooterArmMinAngle+2){
//    		addSequential(new ShooterArmMoveToSetLocation(RobotMap.shooterArmBallLoadAngle)); 
//    	}
    	addParallel(new IntakeSetToSpeed(1));
    	addParallel(new FlyWheelSetToSpeed(-1500));
    	addSequential(new WaitForBallLoaded());
    	addSequential(new WaitSeconds(.5));
    	addParallel(new IntakeSetToSpeed(0)); 
    	addSequential(new FlyWheelStop());
    	    	
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
