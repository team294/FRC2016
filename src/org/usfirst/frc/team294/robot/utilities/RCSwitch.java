package org.usfirst.frc.team294.robot.utilities;

import edu.wpi.first.wpilibj.PWM;

public class RCSwitch {
    private class RCSwitchPWM extends PWM {
        public RCSwitchPWM(int channel) {
            super(channel);
            setBounds(2.1, 1.55, 1.5, 1.45, 0.9);
            setPeriodMultiplier(PeriodMultiplier.k1X);
            setRaw(0);
            setZeroLatch();
        }
    }
    RCSwitchPWM pwm;
    
    public RCSwitch(int channel) {
        pwm = new RCSwitchPWM(channel);
    }
    
    public void set(boolean on) {
        pwm.setRaw(on ? 255 : 0);
    }
}
