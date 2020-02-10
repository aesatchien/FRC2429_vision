package org.usfirst.frc.team2429.spartan2017.movement;

import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.RobotMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SitStill extends Command {

	private int executionCount;
	private double timeout;
	
	public SitStill(double timeout) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.drivetrain);
		setTimeout(timeout);
		this.timeout = timeout;
	}

	public SitStill() {
		requires(Robot.drivetrain);
		setTimeout(0.1);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		System.out.println("SitStill called at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    	executionCount = 0;
    	SmartDashboard.putString("Current Command", this.getClass().getSimpleName());
    	SmartDashboard.putNumber("Parameter", timeout);

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
    	executionCount++;
    	SmartDashboard.putNumber("Iterations", executionCount);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
			return isTimedOut();
	}

	// Called once after isFinished returns true
	protected void end() {

		System.out.println("SitStill ended at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s after running " + String.format("%.2f", timeSinceInitialized()) + "s" );
    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		System.out.println("SitStill interrupted at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);

	}
}
