package org.usfirst.frc.team2429.spartan2017.commands;

import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class AutonomousSetImagingVariables extends InstantCommand {
	
	private NetworkTable gearTable;
	private NetworkTable shooterTable;
    
	public AutonomousSetImagingVariables() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.imageProcessor);
    }

    // Called once when the command executes
    protected void initialize() {
    	System.out.println("AutoSetImagingVars started at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    	gearTable = NetworkTable.getTable("GearCam");
		shooterTable = NetworkTable.getTable("ShooterCam");
		Timer.delay(0.02);
		
	
		//Still not sure which of these distances I want - I may want the other one
		//And they should be set by SensorUpdate() anyway
		Robot.imageProcessor.autonomousGearDistance= gearTable.getNumber("distanceByHeight", -1.0);
		Robot.imageProcessor.autonomousGearStrafe = gearTable.getNumber("strafe", -1.0);
		Robot.imageProcessor.autonomousGearRotation = gearTable.getNumber("rotation", -1.0);
		
		Robot.imageProcessor.autonomousShooterRotation =shooterTable.getNumber("rotation", -1.0);
		Robot.imageProcessor.autonomousShooterDistance = shooterTable.getNumber("distance", -1.0);
		
		//This is the one that should work
		Robot.imageProcessor.setStrafeCorrection();
		
		Timer.delay(0.02);
		
    }

}
