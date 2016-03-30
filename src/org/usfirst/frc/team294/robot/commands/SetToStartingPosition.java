package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;



public class SetToStartingPosition extends CommandGroup {
	
	public SetToStartingPosition() {
		
		addSequential(new IntakeRaiseWithArmMoveIfNeeded());
	}

}
