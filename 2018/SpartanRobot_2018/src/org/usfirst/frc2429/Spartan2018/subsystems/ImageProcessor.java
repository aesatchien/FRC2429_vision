package org.usfirst.frc2429.Spartan2018.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ImageProcessor extends Subsystem {

	int updateCounter = 0;
	public static boolean bCubecamConnected = false;
	public static double targetsAcquired;
	public static double cubecamDistance;
	public static double cubecamAngle;
	
	static NetworkTableInstance inst;
	static NetworkTable table;
	static NetworkTableEntry rotationEntry;
	static NetworkTableEntry connectedEntry;
	static NetworkTableEntry targetsEntry;
	static NetworkTableEntry distanceEntry;
	
	public ImageProcessor () {
		super();
    	//Connect to the network table and get the cubecam key
    	inst = NetworkTableInstance.getDefault();
    	table = inst.getTable("CubeCam");
		connectedEntry = table.getEntry("connected");
		targetsEntry = table.getEntry("targets");
	    distanceEntry = table.getEntry("distance");
	    rotationEntry = table.getEntry("rotation");
	    bCubecamConnected = connectedEntry.getBoolean(false);
		
	}

    public void initDefaultCommand() {

    }
    
    
    public boolean isCubecamConnected(){
     	bCubecamConnected = connectedEntry.getBoolean(false);
    	return bCubecamConnected;
    }
    
    public double getTargetsAcquired(){
    	targetsAcquired = targetsEntry.getDouble(0.0);
    	return targetsAcquired;
    }
    
    public double getCubecamAngle(){
    	cubecamAngle = rotationEntry.getDouble(0.0);
    	return cubecamAngle;
    }
    
    public double getCubecamDistance(){
    	cubecamDistance=distanceEntry.getDouble(0.0);
    	return cubecamDistance;
    }
    
	public void log() {
		
		updateCounter++;
		if (updateCounter%10 == 0) {
		//Update all of the camera values so we can check them on the dashboard	
		SmartDashboard.putBoolean("CubeCam", isCubecamConnected());
		SmartDashboard.putNumber("Targets", getTargetsAcquired());
		SmartDashboard.putNumber("Cubecam Distance", getCubecamDistance());
		SmartDashboard.putNumber("Cubecam Angle", getCubecamAngle());
		}
	}
    
}

