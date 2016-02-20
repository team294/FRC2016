package org.usfirst.frc.team294.robot.utilities;

/**
 * Class used to check if a measured value is within a given
 * tolerance for a given number of measurements.
 * 
 * @author Don Sawdai
 *
 */
public class ToleranceChecker {
	double tolerance;
    int requiredInToleranceSamples;    	// These number of samples must be within tolerance for "success"
    int nInToleranceSamples;  			// Number of successive measurements that were in tolerance

    /**
     * Create a new tolerance checker
     * @param tolerance abs(error) must be less than or equal to tolerance for measurement to be "good" 
     * @param requiredInToleranceSamples number of measured samples that must be in tolerance for "success"
     */
    public ToleranceChecker(double tolerance, int requiredInToleranceSamples) {
    	this.tolerance = tolerance;
    	this.requiredInToleranceSamples = requiredInToleranceSamples;
    	reset();
    }
    
    /**
     * Reset tolerance checker to "out of tolerance"
     */
    public void reset() {
    	nInToleranceSamples = 0;
    }
    
    /**
     * Check if error is within tolerance
     * @param error value to check
     */
    public void check(double error) {
    	if (Math.abs(error)<=tolerance) {
    		nInToleranceSamples++;
    	} else {
    		nInToleranceSamples = 0;
    	}
    }
    
    /**
     * Returns whether or not the last N error samples been within tolerance
     * @return true = yes, false = no
     */
    public boolean success() {
    	return (nInToleranceSamples >= requiredInToleranceSamples);
    }
    
    /**
     * Check if current error is within tolerance, then returns whether or not the 
     * last N error samples been within tolerance
     * @param error current error
     * @return true = yes, false = no
     */
    public boolean success(double error) {
    	check(error);
    	return success();
    }
}
