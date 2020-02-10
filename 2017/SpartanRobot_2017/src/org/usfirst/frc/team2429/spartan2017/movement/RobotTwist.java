package org.usfirst.frc.team2429.spartan2017.movement;

import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RobotTwist extends Command {

	public boolean isOperatorEnabled;
	private double target;
	private double kError;
	private double speed;
	private int executionCount;
	
	//private double startingAngle;

	public RobotTwist(double target) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.drivetrain);
		
		double absoluteSpeed;
		isOperatorEnabled = false;
		absoluteSpeed = Robot.drivetrain.autonomousTwistSpeed;
		
		//Three degrees seems about right to control overshoot at moderate twist speeds
		kError = 3;
		this.target = target;
		if(target > 0){
			speed = absoluteSpeed;
		}else if(target < 0){
			speed = -absoluteSpeed;
		}else{
			
		}
		
		//We had two ways of doing this - reset the gyro or take the current value
		//For some reason this does not work the second time through - breaks after 1st use
		//so I'm sticking with the resetting of the gyro for now
		//startingAngle = RobotMap.gyro.getAngle();
		
		RobotMap.gyro.reset();

	}

	public RobotTwist() {
		// Use requires() here to declare subsystem dependencies
		isOperatorEnabled = true;
		requires(Robot.drivetrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		System.out.println("RobotTwist called at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    	executionCount = 0;
    	SmartDashboard.putString("Current Command", this.getClass().getSimpleName());
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		
		RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, speed, 0);
		
    	executionCount++;
    	SmartDashboard.putNumber("Iterations", executionCount);
    	SmartDashboard.putNumber("Parameter", target);

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
		System.out.println("RobotTwist ended at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		System.out.println("RobotTwist interrupted at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);

	}
}