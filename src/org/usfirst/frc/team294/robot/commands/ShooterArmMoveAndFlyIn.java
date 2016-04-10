package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ShooterArmMoveAndFlyIn extends Command {
    Command flywheel, arm;
    public ShooterArmMoveAndFlyIn(RobotMap.ShootFromLocation location) {
    	flywheel = new FlyWheelSetToSpeedIfArmIsLow(-500);
    	arm = new ShooterArmMoveToSetLocation(location);
    }
    
    public  ShooterArmMoveAndFlyIn(double angle) {
    	flywheel = new FlyWheelSetToSpeedIfArmIsLow(-500);
    	arm = new ShooterArmMoveToSetLocation(angle);
    }
    
    protected void initialize() {
    	flywheel.start();
    	arm.start();
    }
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
