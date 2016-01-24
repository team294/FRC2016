package org.usfirst.frc.team294.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Vision extends Subsystem {
	NetworkTable table;
	double[] defaultValue = new double[0];
	public Vision(){
	  table = NetworkTable.getTable("GRIP/MyContoursReport"); 
	}
    public double[] getLines(){
    double[] lines =  table.getNumberArray("height", defaultValue); 
    
     for (int i=0; i<lines.length; i++){
    	 SmartDashboard.putNumber("lines", lines[i]);
     }
     return lines; 
    }
    //returns the inner most vertical lines in form (x1,x2,x3,y1,y2,y3)
   public double[] getInnerLines(){
	 
	   double[] x1 = table.getNumberArray("x1", defaultValue);
	   for(int i=0; i<angle.length;i++){
	   if(angle[i]>60 && angle[i]<120){
		   if(x1)
	   }
	   }
   }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

