package org.usfirst.frc.team294.robot.triggers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 *
 */
public class AxisTrigger extends Trigger {
    Joystick joystick;
    int axis;
    double min;
    
    public AxisTrigger(Joystick joystick, int axis, double min) {
        this.joystick = joystick;
        this.axis = axis;
        this.min = min;
    }
    
    public boolean get() {
        return joystick.getRawAxis(axis) > min;
    }
}
