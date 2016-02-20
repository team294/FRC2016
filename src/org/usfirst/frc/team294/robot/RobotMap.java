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
	public static int driveTrainGyro1 = 0;

	// RoboRIO digital I/O addresses
	public static int ballSenseButton = 9;

   // Pneumatic controller PCM IDs
    public static int shifterSolenoidFwd = 1;
    public static int shifterSolenoidRev = 0;
//    public static int shooterPistonFwd = 3;  // Old code for double solenoid on prototype bot
//    public static int shooterPistonRev = 2;
    public static int shooterPiston = 3;
    public static int intakeSolenoidFwd = 5;
    public static int intakeSolenoidRev = 6;
    
    // Range of motion for shooter arm
    public static double shooterArmMinAngle=0;
    public static double shooterArmMaxAngle=93.0;
    public static double shooterArmLoadAngle=shooterArmMinAngle;

    // Shooter arm range to avoid when raising or lowering intake
    public static double lowerBoundAngleToAvoid = 25;
    public static double upperBoundAngleToAvoid = 80;
    
    
    public static void init() {
        // Instantiates all objects
    	// Don -- I moved this code into each subsystem!      
    }
}
