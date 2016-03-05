package org.usfirst.frc.team294.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;

import org.usfirst.frc.team294.robot.commands.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());


    // Joystick controls
    public Joystick leftJoystick = new Joystick(0);
    public Joystick rightJoystick = new Joystick(1);
    public Joystick coPanel = new Joystick(2);
    public Joystick coJoystick = new Joystick(3);
    
    // Joystick buttons
    Button[] left = new Button[15];
    Button[] right = new Button[15];
    Button[] coP = new Button[15];
    Button[] coJ = new Button[15];

    public OI() {
        // Create buttons
        for (int i=1; i<15; i++) {
            left[i] = new JoystickButton(leftJoystick, i);
            right[i] = new JoystickButton(rightJoystick, i);
            coP[i] = new JoystickButton(coPanel, i);
            coJ[i] = new JoystickButton(coJoystick, i);
        }

        left[1].whenPressed(new ShiftDown());
        right[1].whenPressed(new ShiftUp());

        coP[4].whenPressed(new ShootBall());
        coP[6].whenPressed(new ShooterArmMoveToSetLocation(RobotMap.shootingAngle));
        coP[7].whenPressed(new ShooterArmMoveToSetLocation(RobotMap.shootingAngle));
        coP[3].whenPressed(new ShooterArmMoveToSetLocation(0));
        coP[12].whenPressed(new LoadBallSequence());
        coP[9].whenPressed(new IntakeRaise());
        coP[11].whenPressed(new IntakeLower());
        coP[13].whenPressed(new IntakeSetToSpeed(-1));
        coP[14].whenPressed(new IntakeSetToSpeed(0));
        
        // SmartDashboard Buttons
        SmartDashboard.putData("Autonomous Command", new AutonomousCommandGroup());
        
        SmartDashboard.putData("DriveWithJoysticks", new DriveWithJoysticks());
        SmartDashboard.putData("DriveTalon: driveForward 5 revs", new DriveDistance(1.0, 5.0));
        SmartDashboard.putData("DriveTalon: driveForward 1 rev", new DriveDistance(0.5, 1.0));
        SmartDashboard.putData("DriveStraightNxp: 5 revs fast", new DriveStraightDistance(0.75, 5.0));
        SmartDashboard.putData("DriveStraightNxp: 5 revs slow", new DriveStraightDistance(0.4, 5.0));
        SmartDashboard.putData("DriveStraightNxp: 1 rev fast", new DriveStraightDistance(0.75, 1.0));
        SmartDashboard.putData("DriveStraightNxp: 1 rev slow", new DriveStraightDistance(0.4, 1.0));
        SmartDashboard.putData("Rotate +90 PID", new DriveAnglePID(90.0));
        SmartDashboard.putData("Rotate -90 PID", new DriveAnglePID(-90.0));
        SmartDashboard.putData("Rotate +5 PID", new DriveAnglePID(5.0));
        SmartDashboard.putData("Rotate -5 PID", new DriveAnglePID(-5.0));
        SmartDashboard.putData("Rotate +2 PID", new DriveAnglePID(2.0));
        SmartDashboard.putData("Rotate -2 PID", new DriveAnglePID(-2.0));
        SmartDashboard.putData("Stop", new DriveStop());
        SmartDashboard.putData("ShiftUp", new ShiftUp());
        SmartDashboard.putData("ShiftDown", new ShiftDown());
        
        SmartDashboard.putData("Shoot ball", new ShootBall());
        SmartDashboard.putData("Piston out", new ShooterPistonOut(true));
        SmartDashboard.putData("Piston in", new ShooterPistonOut(false));
        SmartDashboard.putData("Start FlyWheels", new FlyWheelSetToSpeed(4500));
        SmartDashboard.putData("Intake FlyWheels", new FlyWheelSetToSpeed(-2500));
        SmartDashboard.putData("Start Top FlyWheel only", new FlyWheelSetToSpeed(4500, 0));
        SmartDashboard.putData("Start Bottom FlyWheel only", new FlyWheelSetToSpeed(0, 4500));
        SmartDashboard.putData("Stop FlyWheels", new FlyWheelStop());
        
        SmartDashboard.putData("Intake Raise", new IntakeRaiseWithArmMoveIfNeeded());
        SmartDashboard.putData("Intake Lower", new IntakeLower());
        SmartDashboard.putData("Intake Motor Stop", new IntakeMotorStop());
        SmartDashboard.putData("Intake Rollers In", new IntakeSetToSpeed(1));
        SmartDashboard.putData("Intake Rollers Out", new IntakeSetToSpeed(-1));
        
        SmartDashboard.putData("Shooter Arm 90", new ShooterArmMoveToSetLocation(90));
        SmartDashboard.putData("Shooter Arm 78", new ShooterArmMoveToSetLocation(78));
        SmartDashboard.putData("Shooter Arm 75.5", new ShooterArmMoveToSetLocation(75.5));
        SmartDashboard.putData("Shooter Arm 73", new ShooterArmMoveToSetLocation(73));
        SmartDashboard.putData("Shooter Arm 135", new ShooterArmMoveToSetLocation(135));
        SmartDashboard.putData("Shooter Arm 0", new ShooterArmMoveToSetLocation(0));
        //REMOVE THIS BUTTON ONLY FOR TESTING.
        //SmartDashboard.putData("Shooter Arm -50", new ShooterArmMoveToSetLocation(-50));
        SmartDashboard.putData("Shooter Arm Joystick Relative", new ShooterArmMoveRelativeJoystick());
        SmartDashboard.putData("Shooter Arm keep out!", new ShooterArmMoveAwayFromIntake());
        
        SmartDashboard.putData("Load Ball", new LoadBallSequence());
    }
}

