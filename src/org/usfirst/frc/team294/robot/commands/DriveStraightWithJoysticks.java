package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraightWithJoysticks extends Command {
	
	Joystick joystickInControl;
	
    public DriveStraightWithJoysticks(Joystick joystick) {
        requires(Robot.driveTrain);
        joystickInControl = joystick;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.setDriveControlByPower();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveTrain.driveWithJoystick(joystickInControl, joystickInControl);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.driveTrain.stop();
    }
}
