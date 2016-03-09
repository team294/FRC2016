package org.usfirst.frc.team294.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ShooterPistonOverride extends CommandGroup {
    
    public ShooterPistonOverride() {
    	addSequential(new ShooterPistonOut(true));
    	addSequential(new WaitSeconds(.1));
    	addSequential(new ShooterPistonOut(false));
    }
}
