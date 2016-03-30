package org.usfirst.frc.team294.robot.triggers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 *
 */
public class POVTrigger extends Trigger {
    Joystick joystick;
    int pov;
    int val;
    
    public POVTrigger(Joystick joystick, int pov, int val) {
        this.joystick = joystick;
        this.pov = pov;
        this.val = val;
    }
    public POVTrigger(Joystick joystick, int val) {
        this.joystick = joystick;
        this.pov = 0;
        this.val = val;
    }
    public boolean get() {
        return joystick.getPOV(pov) == val;
    }
}
