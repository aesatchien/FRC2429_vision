package org.usfirst.frc.team2429.spartan2017.commands;

import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.RobotMap;
import org.usfirst.frc.team2429.spartan2017.subsystems.ImageProcessor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SensorUpdate extends Command {
	
	private NetworkTable gearTable;
	private NetworkTable shooterTable;
	private boolean bDebugging = false;
	

    public SensorUpdate() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("SensorUpdate started at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		gearTable = NetworkTable.getTable("GearCam");
		shooterTable = NetworkTable.getTable("ShooterCam");
		
		//Call once here, but not when repeating
		//SmartDashboard.putBoolean("ShootingAllowed", Robot.oi.isShootingAllowed());
		
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	RobotMap.gyroCounter++;
    	
		//Smartdashboard gyro updatees
		SmartDashboard.putNumber("Gyro Angle", (double) ((int) (RobotMap.gyro.getAngle() * 100)) / 100.0);
		
		//encoder and sensor  updates
    	SmartDashboard.putNumber("Distance", (double) ((int)(RobotMap.wheelEncoder.getDistance()* 10)) / 10.0);
    	SmartDashboard.putBoolean("GearPresent",Robot.drivetrain.isGearPresent());
    	
    	//SmartDashboard.putNumber("Shooter Speed", (int)Robot.ballShooter.ballShooterMotor.getSpeed());
		//SmartDashboard.putNumber("Shooter Output Voltage", (int)(100*Robot.ballShooter.ballShooterMotor.getOutputVoltage())/100.);
		SmartDashboard.putNumber("UltraSonic", (double) ((int)(RobotMap.gearUltra.getVoltage()* 10)) / 10.0);
		SmartDashboard.putNumber("POV Reading", Robot.oi.xbox1.getPOV());
		
    	if (bDebugging){
    		RobotMap.gyroArray[RobotMap.gyroCounter%10] = RobotMap.gyro.getAngle();
    		RobotMap.gyroRateArray[RobotMap.gyroCounter%10] = RobotMap.gyro.getRate();
        	RobotMap.gyroRateAverage = 0.0;
        	RobotMap.gyroAverage = 0.0;
    		for (int i = 0; i < 10; i++) {
    			RobotMap.gyroRateAverage += RobotMap.gyroRateArray[i] * 0.1;
    			RobotMap.gyroAverage += RobotMap.gyroArray[i] * 0.1;
    		}
	    	SmartDashboard.putNumber("Rate", RobotMap.wheelEncoder.getRate());
	    	SmartDashboard.putBoolean("Direction", RobotMap.wheelEncoder.getDirection());
	    	SmartDashboard.putBoolean("Stopped", RobotMap.wheelEncoder.getStopped());
	    	//SmartDashboard.putNumber("Ultra Voltage", RobotMap.gearUltra.getVoltage());
			//SmartDashboard.putNumber("Gyro Angle Average", (double) ((int) (RobotMap.gyroAverage * 1000)) / 1000.0);
			//SmartDashboard.putNumber("Gyro Rate Average", (double) ((int) (RobotMap.gyroRateAverage * 1000)) / 1000.0);


		}
		
		//Do this if we are being served information by networkTables
		if (ImageProcessor.offBoardImaging)
		{
			Robot.imageProcessor.gearTargetCount = gearTable.getNumber("targets", -1.0);
			Robot.imageProcessor.gearTargetDistance= gearTable.getNumber("distanceByHeight", 0);
			Robot.imageProcessor.gearTargetStrafe = gearTable.getNumber("strafe", 0);
			Robot.imageProcessor.gearTargetRotation = gearTable.getNumber("rotation", 0);
			Robot.imageProcessor.shooterTargetDistance = shooterTable.getNumber("distance", 0);
			Robot.imageProcessor.shooterTargetRotation = shooterTable.getNumber("rotation", 0);
			Robot.imageProcessor.isGearcamConnected = gearTable.getBoolean("connected",false);
			Robot.imageProcessor.isShootercamConnected = shooterTable.getBoolean("connected",false);
			SmartDashboard.putBoolean("ShooterCam", Robot.imageProcessor.isShootercamConnected);
			SmartDashboard.putBoolean("GearCam", Robot.imageProcessor.isGearcamConnected);
			
			
			if (bDebugging){
			SmartDashboard.putNumber("Shooter Distance", (int)(100*Robot.imageProcessor.shooterTargetDistance)/100.);
			SmartDashboard.putNumber("Shooter Rotation", (int)(100*Robot.imageProcessor.shooterTargetRotation)/100.);
			SmartDashboard.putNumber("rPi Targets", (int)(100*Robot.imageProcessor.gearTargetCount)/100.);
			SmartDashboard.putNumber("rPi Distance",(int)(100*Robot.imageProcessor.gearTargetDistance)/100.);
			SmartDashboard.putNumber("rPi Strafe", (int)(100*Robot.imageProcessor.gearTargetStrafe)/100.);
			SmartDashboard.putNumber("rPi Rotation", (int)(100*Robot.imageProcessor.gearTargetRotation)/100.);
			}
		}
	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("SensorUpdate ended at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("SensorUpdate interrupted at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    }
}
