package org.usfirst.frc.team294.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team294.robot.commands.*;
import org.usfirst.frc.team294.robot.triggers.AxisTrigger;
import org.usfirst.frc.team294.robot.triggers.POVTrigger;


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

	//Threshold position checker
	//For top and bottom knobs unless otherwise specified
	double[] knobThreshold=new double[]{-0.911,-0.731,-0.551,-0.367,-0.1835,-0.0035,0.1775,0.3605,0.5455,0.7285,0.91};
	//For middle knob only
	double[] middleKnobThreshold=new double[] {-0.751,-0.25,0.2525,0.7525};

	public enum TopKnob {
		minus6degrees,minus5degrees,minus4degrees,minus2degrees,minus3degrees,minus1degree,noChange,
		plus1degree, plus2degrees,plus3degrees,plus4degrees,plus5degrees
	}
	public enum MiddleKnob{
		PositionOne,PositionTwo,PositionThree,PositionFour,PositionFive
	}
	public enum BottomKnob {
		LowBar,Portcullis,Ramparts,Moat,RockWall, RoughTerrain,DrawBridge,SallyPort,ChevalDeFrise,noChange
	}
	TopKnob[] TopKnobPositions = new TopKnob[] {TopKnob.minus6degrees, TopKnob.minus5degrees, TopKnob.minus4degrees, TopKnob.minus3degrees,
			TopKnob.minus2degrees, TopKnob.minus1degree, TopKnob.noChange, TopKnob.plus1degree, TopKnob.plus2degrees, TopKnob.plus3degrees, TopKnob.plus4degrees,
			TopKnob.plus5degrees};
	MiddleKnob[] MiddleKnobPositions = new MiddleKnob[] {MiddleKnob.PositionOne, MiddleKnob.PositionTwo, MiddleKnob.PositionThree,MiddleKnob.PositionFour, MiddleKnob.PositionFive};
	BottomKnob[] BottomKnobPositions= new BottomKnob[] {BottomKnob.Portcullis, BottomKnob.ChevalDeFrise, BottomKnob.Ramparts,BottomKnob.Moat,
			BottomKnob.DrawBridge, BottomKnob.SallyPort, BottomKnob.RockWall, BottomKnob.RoughTerrain, BottomKnob.LowBar, BottomKnob.noChange, BottomKnob.noChange
	};
	
	Command[] MiddleKnobCommands = new Command[] {
		null,		// Position 1
		new AutoDriveAndShootPosition2(),		// Position 2
		new AutoDriveAndShootPosition3(),		// Position 3
		new AutoDriveAndShootPosition4(),		// Position 4
		new AutoDriveAndShootPosition5()		// Position 5
	};
	
	Command[] BottomKnobCommands = new Command[] {	
		new AutoPortcullis(), 		//Portcullis 
		new AutoCheval(), 		//ChevalDeFrise 
		new AutoFastBarrier(), 		//Ramparts
		new AutoFastBarrier(), 		//Moat
		new AutoCruiseConfig(), 		//DrawBridge 
		new AutoCruiseConfig(), 		//SallyPort
		new AutoFastBarrier(), 		//RockWall 
		new AutoFastBarrier(), 		//RoughTerrain 
		new AutoLowBar(), 		//LowBar
		new AutoCruiseConfig(), 		//noChange 
		new AutoCruiseConfig() 		//noChange
	};

	// Joystick controls
	public Joystick leftJoystick = new Joystick(0);
	public Joystick rightJoystick = new Joystick(1);
	public Joystick coPanel = new Joystick(2);
	public Joystick xboxController = new Joystick(3);
	public Joystick coJoystick = new Joystick(4);
	
	public OI() {
		Button[] left = new Button[12];
	    Button[] right = new Button[12];
	    Button[] coP = new Button[15];
	    Button[] coJ = new Button[12];
	    Button[] xbB = new Button[11];
	    Trigger xbLT = new AxisTrigger(xboxController, 2, 0.9);
        Trigger xbRT = new AxisTrigger(xboxController, 3, 0.9);
        Trigger xbPovUp = new POVTrigger(xboxController, 0);
        Trigger xbPovRight = new POVTrigger(xboxController, 90);
        Trigger xbPovDown = new POVTrigger(xboxController, 180);
        Trigger xbPovLeft = new POVTrigger(xboxController, 270);
		
		// Create buttons
		for (int i=1; i<left.length; i++) {
			left[i] = new JoystickButton(leftJoystick, i);
			right[i] = new JoystickButton(rightJoystick, i);
			if (i==2) {
				left[2].whenPressed(new DriveWithJoysticks());
				right[2].whenPressed(new AutoTargetShoot());
			} else if(i==3) {
				left[i].whileHeld(new DriveStraightWithJoysticks(leftJoystick));
				right[i].whileHeld(new DriveStraightWithJoysticks(rightJoystick));
			} else {
				left[i].whenPressed(new ShiftDown());
				right[i].whenPressed(new ShiftUp()); 
			}
		}
		for (int i=1; i<coP.length; i++) {
		    coP[i] = new JoystickButton(coPanel, i);
		}
		for (int i=1; i<coJ.length; i++) {
		    coJ[i] = new JoystickButton(coJoystick, i);
		}
		for (int i=1; i<xbB.length; i++) {
		    xbB[i] = new JoystickButton(xboxController, i);
		}

		xbB[1].whenPressed(new IntakeLowerIfRaised());		
		xbB[2].whenPressed(new IntakeSetToSpeed(-1));
		xbB[3].whenPressed(new LoadBallSequence());
		xbB[4].whenPressed(new IntakeRaiseWithArmMoveIfNeeded());
		xbB[5].whenPressed(new ShooterArmMoveAndStopFlywheels(RobotMap.shooterArmBallCruiseAngle));
		xbB[6].whenPressed(new ShooterArmMoveAndFlyIn(RobotMap.ShootFromLocation.Batter));
		//add code so while xbB[7] is held, the arm piston can be moved e.g. it is enabled
		xbB[8].whileHeld(new IntakeOverride(true));
		xbB[8].whenReleased(new IntakeOverride(false));
	    xbB[9].whenPressed(new StopFlyAndIntake());
		//add code so when xbB[10] is pressed, the arm piston moves

        xbPovUp.whenActive(new ShooterArmMoveAndFlyIn(RobotMap.ShootFromLocation.Outerworks));
        xbPovDown.whenActive(new ShooterArmMoveAndStopFlywheels(RobotMap.shooterArmBallLoadAngle));
		xbPovLeft.whenActive(new ShooterArmMoveAndFlyIn(RobotMap.ShootFromLocation.EndOfBatter));
		xbPovRight.whenActive(new ShooterArmMoveAndFlyIn(RobotMap.ShootFromLocation.ManualOuterworks));

		//Rev the flywheels to the correct speed
		xbLT.whenActive(new FlyWheelSetToSpeedForArmLocation());
		//This will do the shooting sequence
	    xbRT.whenActive(new ShootBall());

        coP[1].whenPressed(new ShootBall());
        coP[2].whenPressed(new ShooterArmMoveAndFlyIn(RobotMap.ShootFromLocation.Outerworks));
        coP[3].whenPressed(new ShooterArmMoveAndStopFlywheels(RobotMap.shooterArmBallLoadAngle));
        coP[4].whenPressed(new FlyWheelSetToSpeedForArmLocation());
        coP[5].whileHeld(new IntakeOverride(true));
        coP[5].whenReleased(new IntakeOverride(false));
        coP[6].whenPressed(new ShooterArmMoveAndFlyIn(RobotMap.ShootFromLocation.Batter));
        coP[7].whenPressed(new ShooterArmMoveAndStopFlywheels(RobotMap.shooterArmBallCruiseAngle));
        //Set the arm piston to out
        coP[9].whenPressed(new IntakeSetToSpeed(-1));
        //Set the arm piston to in
        coP[11].whenPressed(new LoadBallSequence());
        coP[12].whenPressed(new StopFlyAndIntake());
        coP[13].whenPressed(new IntakeLowerIfRaised());
        coP[14].whenPressed(new IntakeRaiseWithArmMoveIfNeeded());

        //coJ[1].whileHeld(new ShooterArmMoveRelativeJoystick());

		// SmartDashboard Buttons
		SmartDashboard.putData("Debug dashboard", new SmartDashboardDebug());

		SmartDashboard.putData("DriveWithJoysticks", new DriveWithJoysticks());
        SmartDashboard.putData("Drive: All Stop", new DriveStop());
        SmartDashboard.putData("Rotate to goal", new DriveTurnToGoal(1.0));

        SmartDashboard.putData("ShiftUp", new ShiftUp());
        SmartDashboard.putData("ShiftDown", new ShiftDown());
        
        SmartDashboard.putData("Load Ball", new LoadBallSequence());
        SmartDashboard.putData("Shoot ball (old)", new ShootBallSetFlywheels());
        SmartDashboard.putData("Shoot ball", new ShootBall());
        SmartDashboard.putData("Shooter Arm Joystick Relative", new ShooterArmMoveRelativeJoystick());

        SmartDashboard.putData("Piston out", new ShooterPistonOut(true));
        SmartDashboard.putData("Piston in", new ShooterPistonOut(false));
		SmartDashboard.putData("Port Piston Out", new ArmPistonOut());
		SmartDashboard.putData("Port Piston In", new ArmPistonIn());
        SmartDashboard.putData("Start FlyWheels", new FlyWheelSetToSpeed(RobotMap.maxFlywheelSpeed));
        SmartDashboard.putData("Intake FlyWheels", new FlyWheelSetToSpeed(-2500));
        SmartDashboard.putData("Stop FlyWheels", new FlyWheelStop());
        
        SmartDashboard.putData("Intake Raise", new IntakeRaiseWithArmMoveIfNeeded());
        SmartDashboard.putData("Intake Lower", new IntakeLowerIfRaised());
        SmartDashboard.putData("Intake Rollers In", new IntakeSetToSpeed(1));
        SmartDashboard.putData("Intake Rollers Out", new IntakeSetToSpeed(-1));        
        SmartDashboard.putData("Intake Motor Stop", new IntakeMotorStop());

        SmartDashboard.putData("Shooter Arm Shooting Angle", new ShooterArmMoveToSetLocation(RobotMap.shootingAngle));
        SmartDashboard.putData("Shooter Arm 0", new ShooterArmMoveToSetLocation(0));
        SmartDashboard.putData("Shooter Arm Load", new ShooterArmMoveAndStopFlywheels(RobotMap.shooterArmBallLoadAngle));
        SmartDashboard.putData("Shooter Arm Cruise", new ShooterArmMoveAndStopFlywheels(RobotMap.shooterArmBallCruiseAngle));
        SmartDashboard.putData("Shooter Arm OuterWks", new ShooterArmMoveAndRev(RobotMap.shootingAngleFromOuterworks, RobotMap.maxFlywheelSpeed, RobotMap.maxFlywheelSpeed));
        SmartDashboard.putData("Shooter Arm Batter+1", new ShooterArmMoveAndRev(RobotMap.shootingAngleFromEndOfBatter, RobotMap.maxFlywheelSpeed, RobotMap.maxFlywheelSpeed));
        SmartDashboard.putData("Shooter Arm Batter", new ShooterArmMoveAndRev(RobotMap.shootingAngle,2100, 2520));

        SmartDashboard.putData("Shooter Arm Joystick Relative", new ShooterArmMoveRelativeJoystick());
        SmartDashboard.putData("Shooter Arm cam to goal", new ShooterArmMoveToGoal());

        SmartDashboard.putData("Rotate +90", new DriveAngle(0.65, 90, true));
        SmartDashboard.putData("Rotate -90", new DriveAngle(0.65, -90, true));
        SmartDashboard.putData("Rotate +2", new DriveAngle(0.65, +2, true, 0.75));
        SmartDashboard.putData("Rotate -2", new DriveAngle(0.65, -2, true, 0.75));
        SmartDashboard.putData("Rotate +8", new DriveAngle(0.65, +8, true, 2.0));
        SmartDashboard.putData("Rotate -8", new DriveAngle(0.65, -8, true, 2.0));
        SmartDashboard.putData("Rotate +90 fast", new DriveAngle(1.0, 90, true));
        SmartDashboard.putData("Rotate -90 fast", new DriveAngle(1.0, -90, true));

        if (Robot.smartDashboardDebug) {
        	setupSmartDashboardDebug();
        }
    }
    
	public TopKnob readTopKnob() {
		double knobReading;
		int i=0;

		knobReading = coPanel.getRawAxis(4);
		int len=knobThreshold.length;
		for(i=0;i<len; i++) {
			if (knobReading<knobThreshold[i]) break;
		}

        if (Robot.smartDashboardDebug) {
        	SmartDashboard.putNumber("Top Knob Position", i);
        	SmartDashboard.putNumber("Knob Reading", knobReading);
        }
        
		if(i==len)return TopKnobPositions[len-1];
		return TopKnobPositions[i];
	}

	/**
	 * Reads the middle knob.
	 * @return Raw position 0 (full ccw) to 4 (cw).  Positions above 4 are indeterminate due to resistors missing.
	 */
	public int readMiddleKnobRaw() {
		double knobReading2;

		int i=0;
		knobReading2 = coPanel.getRawAxis(6);
		int len=middleKnobThreshold.length;
		for(i=0;i<len; i++) {
			if (knobReading2<middleKnobThreshold[i]) break;
		}

        if (Robot.smartDashboardDebug) {
        	SmartDashboard.putNumber("Middle Knob Position", i);
        	SmartDashboard.putNumber("Middle Knob Reading", knobReading2);
        }

		return i;
	}
	
	/**
	 * Reads the middle knob.
	 * @return OI.MiddleKnob robot starting position constant for the current knob position
	 */
	public MiddleKnob readMiddleKnob(){
		return MiddleKnobPositions[readMiddleKnobRaw()];
	}

	/**
	 * Gets "drive, turn, and shoot" command based on the robot starting position, as
	 * per the middle knob setting.  Use this after crossing the outer works (barriers). 
	 * @return  Command to turn and shoot
	 */
	public Command getMiddleKnobCommand() {
		int i;
		
		i = readMiddleKnobRaw();
		if (i<MiddleKnobCommands.length) {
			return MiddleKnobCommands[i];
		} else {
			return null;
		}			
	}

	/**
	 * Reads the bottom knob.
	 * @return Raw position 0 (full ccw) to 11 (full cw)
	 */
	public int readBottomKnobRaw() {
		double knobReading;
		int i=0;
		
		knobReading = coPanel.getRawAxis(3);
		int len=knobThreshold.length;
		for(i=0;i<len; i++) {
			if (knobReading<knobThreshold[i]) break;
		}
		
        if (Robot.smartDashboardDebug) {
        	SmartDashboard.putNumber("Bottom Knob Position", i);
        	SmartDashboard.putNumber("Knob Reading", knobReading);
        }
        
		if(i==len)return (len-1);
		return (i);
	}

	/**
	 * Reads the bottom knob.
	 * @return OI.BottomKnob barrier constant for the current knob position
	 */
	public BottomKnob readBottomKnob() {
		return BottomKnobPositions[readBottomKnobRaw()];
	}
	
	/**
	 * Gets autonomous command to run based on bottom knob setting (barrier to cross)
	 * @return Command to cross the barrier
	 */
	public Command getAutonomousCommand() {
		return BottomKnobCommands[readBottomKnobRaw()];
	}

    public void updateSmartDashboard() {
    	int i;
    	
    	for (i=0; i<coPanel.getAxisCount(); i++) {
        	SmartDashboard.putNumber("CoPanel Axis " + i, coPanel.getRawAxis(i));
    	}    	
    }
    
    public void setupSmartDashboardDebug() {
    	// Show sub-system data
		Robot.shooter.setupSmartDashboard(false);
		Robot.shooterArm.setupSmartDashboard(false);
		
		SmartDashboard.putData("AutoLowBar",new AutoLowBar());
		SmartDashboard.putData("AutoFastBarrier",new AutoFastBarrier());		

        SmartDashboard.putData("Shoot ball Low", new ShootBallLow());
        SmartDashboard.putData("Shoot ball Cruise", new ShootBallMoveArmLow());
        SmartDashboard.putData("Shoot ball only", new ShootBallOnly());
        SmartDashboard.putData("Shoot ball auto target", new AutoTargetShoot());

        // SmartDashboard Buttons
        SmartDashboard.putData("DriveStraightNxp: 10 revs fast", new DriveStraightDistance(1.0, 10.0, DriveStraightDistance.Units.rotations));
        SmartDashboard.putData("DriveStraightNxp: 10 revs slow", new DriveStraightDistance(0.6, 10.0, DriveStraightDistance.Units.rotations));
        SmartDashboard.putData("DriveStraightNxp: 5 revs fast", new DriveStraightDistance(1.0, 5.0, DriveStraightDistance.Units.rotations));
        SmartDashboard.putData("DriveStraightNxp: 5 revs slow", new DriveStraightDistance(0.6, 5.0, DriveStraightDistance.Units.rotations));
//        SmartDashboard.putData("DriveStraightNxp: 1 rev fast", new DriveStraightDistance(1.0, 1.0, DriveStraightDistance.Units.rotations));
//        SmartDashboard.putData("DriveStraightNxp: 1 rev slow", new DriveStraightDistance(0.6, 1.0, DriveStraightDistance.Units.rotations));
//        SmartDashboard.putData("DriveStraightNxp: -5 revs fast", new DriveStraightDistance(1.0, -5.0, DriveStraightDistance.Units.rotations));
        SmartDashboard.putData("DriveStraightNxp: -5 revs slow", new DriveStraightDistance(0.6, -5.0, DriveStraightDistance.Units.rotations));
        SmartDashboard.putData("DriveStraightNxp: 10 feet slow", new DriveStraightDistance(0.6, 120.0, DriveStraightDistance.Units.inches));
        SmartDashboard.putData("Rotate to 0", new DriveAngle(0.65, 0, false));
//        SmartDashboard.putData("Rotate to 10", new DriveAngle(0.65, 10, false));
//        SmartDashboard.putData("Rotate to 90", new DriveAngle(0.65, 90, false));
        SmartDashboard.putData("Rotate +90", new DriveAngle(0.65, 90, true));
        SmartDashboard.putData("Rotate -90", new DriveAngle(0.65, -90, true));
        SmartDashboard.putData("Rotate +5", new DriveAngle(0.65, +5, true, 0.75));
        SmartDashboard.putData("Rotate -5", new DriveAngle(0.65, -5, true, 0.75));
        SmartDashboard.putData("Rotate +10", new DriveAngle(0.65, +10, true, 2.0));
        SmartDashboard.putData("Rotate -10", new DriveAngle(0.65, -10, true, 2.0));
        SmartDashboard.putData("Rotate +90 fast", new DriveAngle(1.0, 90, true));
        SmartDashboard.putData("Rotate -90 fast", new DriveAngle(1.0, -90, true));


        SmartDashboard.putData("Start Top FlyWheel only", new FlyWheelSetToSpeed(RobotMap.maxFlywheelSpeed, 0));
        SmartDashboard.putData("Start Bottom FlyWheel only", new FlyWheelSetToSpeed(0, RobotMap.maxFlywheelSpeed));

        SmartDashboard.putData("Intake RaiseX", new IntakeRaise());
        SmartDashboard.putData("Intake LowerX", new IntakeLower());

        SmartDashboard.putData("Shooter Arm 90", new ShooterArmMoveToSetLocation(90));
//        SmartDashboard.putData("Shooter Arm 78", new ShooterArmMoveToSetLocation(78));
//        SmartDashboard.putData("Shooter Arm 75.5", new ShooterArmMoveToSetLocation(75.5));
//        SmartDashboard.putData("Shooter Arm 73", new ShooterArmMoveToSetLocation(73));
        SmartDashboard.putData("Shooter Arm 135", new ShooterArmMoveToSetLocation(135));
        SmartDashboard.putData("Shooter Arm 5", new ShooterArmMoveToSetLocation(5));
        SmartDashboard.putData("Shooter Arm 10", new ShooterArmMoveToSetLocation(10));
        SmartDashboard.putData("Shooter Arm 15", new ShooterArmMoveToSetLocation(15));
        //REMOVE THIS BUTTON ONLY FOR TESTING.
        //SmartDashboard.putData("Shooter Arm -50", new ShooterArmMoveToSetLocation(-50));
        SmartDashboard.putData("Shooter Arm keep out!", new ShooterArmMoveAwayFromIntake(ShooterArmMoveAwayFromIntake.condition.ifIntakeNotInWay));
    }
}
