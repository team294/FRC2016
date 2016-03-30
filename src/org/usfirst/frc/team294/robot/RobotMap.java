package org.usfirst.frc.team294.robot;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	//Address for robot hardware

	// CANbus addresses
	public static int driveTrainLeftMotor1 = 5;
	public static int driveTrainLeftMotor2 = 6;
	public static int driveTrainRightMotor1 = 11;
	public static int driveTrainRightMotor2 = 12;
	public static int intakeMotor = 9;
	public static int shooterMotorTop = 20;
	public static int shooterMotorBottom = 21;
	public static int shooterArmMotor=22;

	// RoboRIO analog I/O addresses
//	public static int driveTrainGyro1 = 0;

	// RoboRIO digital I/O addresses
	public static int ballSensor = 0;
	public static int intakeDownSensor = 1;

	// RoboRIO PWM I/O addresses
	public static int flashlight = 0;
    public static int speedlight = 1;

   // Pneumatic controller PCM IDs
    public static int shifterSolenoidFwd = 0;
    public static int shifterSolenoidRev = 1;
    public static int intakeSolenoidFwd = 2;
    public static int intakeSolenoidRev = 3;
    public static int shooterPiston = 4;
    public static int shooterArmBrakeSolenoid = 5;
    
    // Range of motion for shooter arm
    public static double shooterArmMinAngle=0;
    public static double shooterArmMaxAngle=93.0;

    // Shooter arm setpoints
    public static double shooterArmBallLoadAngle = 0.0; 
    public static double shootingAngle = 76; 				//Up on the tower shooting angle
    public static double shooterArmBallCruiseAngle = 10.0;  // Reduced from 15 to 10 degrees due to larger flywheels and arm stops.
    public static double shootingAngleFromOuterworks = 49.0; //Shooting angle from the outerworks according to testing (not implemented yet)
    public static double shootingAngleFromEndOfBatter = 62.0; //Possible shooting angle from the end of the batter (not implemented yet)
    
    // Shooter arm range to avoid when raising or lowering intake
    public static double lowerBoundAngleToAvoid = 6;
    public static double upperBoundAngleToAvoid = 83;
    
    // Shooter flywheel max speed
	//TODO:  Change to 4500 for competition robot;
    public static double maxFlywheelSpeed = 4200;
    
    public static void init() {
        // Instantiates all objects
    	// Don -- I moved this code into each subsystem!      
    }
}
