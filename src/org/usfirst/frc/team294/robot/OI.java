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


    // Creates Joystick variables
    public Joystick leftJoystick;
    public JoystickButton leftJoystickButton2;
    public JoystickButton leftJoystickTrigger;
    
    public Joystick rightJoystick;
    public JoystickButton rightJoystickButtonA;
    public JoystickButton rightJoystickButton4;
    public JoystickButton rightJoystickButton5;
    
    public Joystick thirdJoystick;
    
    public OI() {
        //Instantiates all objects for joysticks and buttons

        rightJoystick = new Joystick(1);
        rightJoystickButtonA = new JoystickButton(rightJoystick, 1);
        rightJoystickButtonA.whileHeld(new DriveDistance(1, 5000));
        rightJoystickButton4 = new JoystickButton(rightJoystick, 4);
        rightJoystickButton4.whileHeld(new ShiftDown());
        rightJoystickButton5 = new JoystickButton(rightJoystick, 5);
        rightJoystickButton5.whileHeld(new ShiftUp());

        leftJoystick = new Joystick(0);
        leftJoystickTrigger = new JoystickButton(leftJoystick, 1);
        leftJoystickTrigger.whenPressed(new ShootBall());
        
        thirdJoystick= new Joystick(2);

        // SmartDashboard Buttons
        SmartDashboard.putData("Autonomous Command", new AutonomousCommandGroup());
        
        SmartDashboard.putData("DriveWithJoysticks", new DriveWithJoysticks());
        SmartDashboard.putData("DriveForward: driveForwardFast", new DriveDistance(1.0, 1.0));
        SmartDashboard.putData("DriveForward: driveForwardSlow", new DriveDistance(0.5, 1.0));
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
        SmartDashboard.putData("Stop FlyWheels", new FlyWheelSetToSpeed(0));
        
        SmartDashboard.putData("Intake Raise", new IntakeRaise());
        SmartDashboard.putData("Intake Lower", new IntakeLower());
        SmartDashboard.putData("Intake Motor Stop", new IntakeMotorStop());
        SmartDashboard.putData("Intake Rollers In", new IntakeSetToSpeed(1));
        SmartDashboard.putData("Intake Rollers Out", new IntakeSetToSpeed(-1));
        
        SmartDashboard.putData("Load Ball", new LoadBallSequence());
    }

    public Joystick getleftJoystick() {
        return leftJoystick;
    }

    public Joystick getrightJoystick() {
        return rightJoystick;
    }
    
}

