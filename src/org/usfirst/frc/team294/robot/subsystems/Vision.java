package org.usfirst.frc.team294.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Vision extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
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
    
/* Code note finished for this method, so it is commented out for now.

   public double[] getInnerLines(){
	   //returns the inner most vertical lines in form (x1,x2,x3,y1,y2,y3)
	   double[] angle = table.getNumberArray("angle", defaultValue);
	   
	   for(int i=0; i<angle.length;i++){
		   if(angle[i]>60 && angle[i]<120){
//			   if(x1)
		   }
	   }
   }
*/
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

