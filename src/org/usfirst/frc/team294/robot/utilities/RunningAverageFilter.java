package org.usfirst.frc.team294.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class used to calculate running averages on incoming data.
 * 
 * @author Don Sawdai
 *
 */
public class RunningAverageFilter {
	int nMax, nCur; 	// Max size and current size
	int iNext;			// Array index of next point to add
	double [] sample;
	double avg;			// Current running average

    /**
     * Create a new running average filter
     * @param N number of samples to average
     */
    public RunningAverageFilter(int N) {
    	nMax = N;
    	sample = new double[N];
    	
    	reset();
    }
    
    /**
     * Reset averaging filter and flush current data
     */
    public void reset() {
    	nCur = 0;
    	iNext = 0;
    	avg = 0;
    }
    
    /**
     * Adds a sample for the running average
     * @param value New value to add to running average
     * @return current running average.  If filter has no data, then returns Double.NaN.  Use Double.isNaN() to check for that value.
     */
    public double add(double value) {
    	if (nCur>=nMax) {
    		// Array is full.  Circularly wrap around array.
    		avg += (value - sample[iNext])/((double)nCur);
    	} else {
    		// Array is not yet full.  Just add point.
    		avg = (avg*((double)nCur) + value) / ((double)(nCur+1));
    		nCur++;
    	}

    	// Save value and set iNext to next free spot
		sample[iNext] = value;
		iNext++;
		iNext = (iNext>=nMax) ? 0 : iNext;
		
		return getAverage();
    }
    
    /**
     * Returns the current running average.  If filter has no data, then returns Double.NaN.  Use
     * Double.isNaN() to check for that value.
     * @return current running average
     */
    public double getAverage() {
    	if (nCur == 0) {
//    		SmartDashboard.putNumber("AvgFilter val", 0.0);
    		return Double.NaN;
    	} else {
//    		SmartDashboard.putNumber("AvgFilter val", avg);
    		return avg;    		
    	}
    }
    
    /**
     * Returns number of points stored in average.
     * @return number of points.  0 = empty.
     */
    public int getNumPoints() {
    	return nCur;
    }
}
