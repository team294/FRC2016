package org.usfirst.frc.team294.robot;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	public static int Left1 = 0;
	public static int Left2 = 1;
	public static int Right1 = 2;
	public static int Right2 = 3;
	public static int Gyro = 0; 
	public static int SolenoidChannel1 = 0;
	public static int SolenoidChannel2 = 1; 
	public static int EncoderLeft = 0;
	public static int EncoderRight = 0; 
	
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    // public static int leftMotor = 1;
    // public static int rightMotor = 2;
    
    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static int rangefinderPort = 1;
    // public static int rangefinderModule = 1;
}
