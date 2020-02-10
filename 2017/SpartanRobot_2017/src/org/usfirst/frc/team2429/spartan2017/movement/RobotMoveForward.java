package org.usfirst.frc.team2429.spartan2017.movement;

import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RobotMoveForward extends Command {

	public boolean isOperatorEnabled;
	double heading;
	double twist;
	private int executionCount;
	
	public RobotMoveForward(double timeout) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.drivetrain);

		isOperatorEnabled = false;
	
		setTimeout(timeout);
		SmartDashboard.putNumber("Parameter", timeout);
	}

	public RobotMoveForward() {

		isOperatorEnabled = true;
		// Use requires() here to declare subsystem dependencies
		requires(Robot.drivetrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		System.out.println("RobotMoveForward called at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		RobotMap.gyro.reset();
    	heading = RobotMap.gyro.getAngle();
    	executionCount = 0;
    	SmartDashboard.putString("Current Command", this.getClass().getSimpleName());
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		twist = Robot.drivetrain.kGyroProportional*(heading - RobotMap.gyro.getAngle());
		RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, - Robot.drivetrain.dpadForwardSpeed, twist, 0);
    	executionCount++;
    	SmartDashboard.putNumber("Iterations", executionCount);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (isOperatorEnabled) {
			return !Robot.oi.buttonPovUp.get();
		} else {
			return isTimedOut();
		}
	}

	// Called once after isFinished returns true
	protected void end() {
		
		if (isOperatorEnabled) {
			System.out.println("RobotMoveForward ended at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		} else {
			System.out.println("RobotMoveForward ended at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s   after running " + String.format("%.2f", timeSinceInitialized()) + "s" );
		}
    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		System.out.println("RobotMoveForward interrupted at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);

	}
}
