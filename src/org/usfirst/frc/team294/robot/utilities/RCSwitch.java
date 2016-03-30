package org.usfirst.frc.team294.robot.utilities;

import edu.wpi.first.wpilibj.Jaguar;

public class RCSwitch {
    Jaguar pwm;
    
    public RCSwitch(int channel) {
        pwm = new Jaguar(channel);
    }
    
    public void set(boolean on) {
        pwm.set(on ? 1 : 0);
    }
}
