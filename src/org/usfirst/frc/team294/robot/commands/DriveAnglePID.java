package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Rotate robot x degrees using navX-mxp
 */
public class DriveAnglePID extends Command {
	double degrees;
	
	/**
	 * Rotate robot x degrees using navX-mxp.  Automatically shifts to low gear.
	 * @param degrees Number of degrees to rotate (+ right, - left (I think))
	 */
    public DriveAnglePID(double degrees) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.driveTrain);

        this.degrees = degrees;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.turnDegreesPIDStart(degrees);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.driveTrain.turnDegreesPIDIsFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.driveTrain.turnDegreesPIDCancel();
    	Robot.driveTrain.stop();
    }
}
