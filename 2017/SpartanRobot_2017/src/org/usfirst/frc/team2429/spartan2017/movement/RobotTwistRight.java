package org.usfirst.frc.team2429.spartan2017.movement;

import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RobotTwistRight extends Command {

	public boolean isOperatorEnabled;
	private double target;
	private double kError;
	private int executionCount;
	//private double startingAngle;

	public RobotTwistRight(double target) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.drivetrain);

		isOperatorEnabled = false;
		kError = 0;
		this.target = target;
		
		//We had two ways of doing this - reset the gyro or take the current value
		//For some reason this does not work the second time through - breaks after 1st use
		//so I'm sticking with the resetting of the gyro for now
		//startingAngle = RobotMap.gyro.getAngle();
		

	}

	public RobotTwistRight() {
		// Use requires() here to declare subsystem dependencies
		isOperatorEnabled = true;
		requires(Robot.drivetrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		System.out.println("RobotTwistRight called at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		RobotMap.gyro.reset();
    	executionCount = 0;
    	SmartDashboard.putString("Current Command", this.getClass().getSimpleName());
    	SmartDashboard.putNumber("Parameter", target);

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, Robot.drivetrain.buttonTwistSpeed, 0);
		
    	executionCount++;
    	SmartDashboard.putNumber("Iterations", executionCount);

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (isOperatorEnabled) {
			return !Robot.oi.buttonB.get();
		} else {
			//return (Math.abs(RobotMap.gyro.getAngle()) - startingAngle) > target - kError;
			return (Math.abs(RobotMap.gyro.getAngle())) > Math.abs(target) - kError;
		}
	}

	// Called once after isFinished returns true
	protected void end() {
		System.out.println("RobotTwistRight ended at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		System.out.println("RobotTwistRight interrupted at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);

	}
}