package org.usfirst.frc.team2429.spartan2017.deprecated;

import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RobotStrafeToGearTarget extends Command {

	public boolean isOperatorEnabled;
	private double distance;
	private double kError;

	public RobotStrafeToGearTarget() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.drivetrain);
	}


	// Called just before this Command runs the first time
	protected void initialize() {
		System.out.println("RobotStrafeToGearTarget called at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		RobotMap.wheelEncoder.reset();
		
		isOperatorEnabled = false;
		
		this.distance = Robot.imageProcessor.gearTargetStrafe;
		kError = 3.0;

		setTimeout(20.0);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, - Robot.drivetrain.autonomousForwardSpeed, 0, 0);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if ((RobotMap.wheelEncoder.getDistance() > distance-kError) || isTimedOut()) {
			return true;
		} else {
			return false;
		}
	}

	// Called once after isFinished returns true
	protected void end() {
		
		if (isOperatorEnabled) {
			System.out.println("RobotStrafeToGearTarget ended at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		} else {
			System.out.println("RobotStrafeToGearTarget ended at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s   after running " + String.format("%.2f", timeSinceInitialized()) + "s" );
		}
    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		System.out.println("RobotStrafeToGearTarget interrupted at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);

	}
}
