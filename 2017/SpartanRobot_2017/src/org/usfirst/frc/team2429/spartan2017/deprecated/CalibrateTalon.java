package org.usfirst.frc.team2429.spartan2017.deprecated;

import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CalibrateTalon extends Command {

    public CalibrateTalon() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("CalibrateTalon called at: " + String.format("%.2f",Timer.getFPGATimestamp())+"s");
    	for(int i =0; i <100; i++){
	    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(1.0, 0, 0, 0);
	    	Timer.delay(0.020);
    	}
    	for(int i =0; i <100; i++){
	    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(-1.0, 0, 0, 0);
	    	Timer.delay(0.020);
    	}
    	for(int i =0; i <100; i++){
	    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 1.0, 0, 0);
	    	Timer.delay(0.020);
    	}
    	for(int i =0; i <100; i++){
	    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, -1.0, 0, 0);
	    	Timer.delay(0.020);
    	} 
    	for(int i =0; i <100; i++){
	    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 1.0, 0);
	    	Timer.delay(0.020);
    	}
    	for(int i =0; i <100; i++){
	    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, -1.0, 0);
	    	Timer.delay(0.020);
    	}
    	for(int i =0; i <100; i++){
	    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(1.0, -1.0, 0, 0);
	    	Timer.delay(0.020);
    	}
    	for(int i =0; i <100; i++){
	    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(-1.0, 1.0, 0, 0);
	    	Timer.delay(0.020);
    	}
    	for(int i =0; i <100; i++){
	    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
	    	Timer.delay(0.020);
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("CalibrateTalon ended at: " + String.format("%.2f",Timer.getFPGATimestamp())+"s");

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("CalibrateTalon interrupted at: " + String.format("%.2f",Timer.getFPGATimestamp())+"s");

    }
}
