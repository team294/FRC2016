package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class IntakeSequence extends CommandGroup {
    
    public  IntakeSequence() {
    	if(Robot.intake.intakeIsUp()){
    		addSequential(new IntakeLower());
    		addSequential(new WaitSeconds(3));
    	} 
    	if(Robot.shooterArm.getAngle()>RobotMap.shooterArmMinAngle){
    		addSequential(new ShooterArmMoveToSetLocation(RobotMap.shooterArmBallLoadAngle)); 
    	}
    	addParallel(new IntakeSetToSpeed(1));
    	addSequential(new IntakeIsButtonPressed());
    	addSequential(new IntakeSetToSpeed(0)); 
    	
    	
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
