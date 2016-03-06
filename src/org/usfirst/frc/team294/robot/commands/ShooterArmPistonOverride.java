package org.usfirst.frc.team294.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ShooterArmPistonOverride extends CommandGroup {
    
    public ShooterArmPistonOverride() {
    	addSequential(new ShooterPistonOut(true));
    	addSequential(new WaitSeconds(.1));
    	addSequential(new ShooterPistonOut(false));
    }
}
