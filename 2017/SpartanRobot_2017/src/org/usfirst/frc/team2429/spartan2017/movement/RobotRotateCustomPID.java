package org.usfirst.frc.team2429.spartan2017.movement;

import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RobotRotateCustomPID extends Command {

	public boolean bUseImaging;
	
	private int executionCount;
	private double setpoint;
	private double kError;
	private double kTolerance = 0.5;
	private double speed;
	
	
	//private double startingAngle;

	public RobotRotateCustomPID(double angle, boolean bUseImaging) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.drivetrain);
				
		this.bUseImaging = bUseImaging;
		this.setpoint = angle;

	}

	

	// Called just before this Command runs the first time
	protected void initialize() {
		
		if (bUseImaging) {
			//setpoint=Robot.imageProcessor.autonomousGearRotation;
			//setpoint = Robot.imageProcessor.gearTargetRotation;
			setpoint = Robot.imageProcessor.shooterTargetRotation;
		} else {
			// was set in constructor
		}
		
		//The farther we go the sooner we need to back off to account for momentum
		//Should be about 0.3 for small angles and 3 for large angles
		kTolerance = 0.2 + 0.07*Math.abs(setpoint);
		
		System.out.println("RobotRotateCustomPID called at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		RobotMap.gyro.reset();
    	executionCount = 0;
    	SmartDashboard.putString("Current Command", this.getClass().getSimpleName());
    	SmartDashboard.putNumber("Parameter", setpoint);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double output;
		
		kError = setpoint - RobotMap.gyro.getAngle();
		output = kError * Robot.drivetrain.kGyroProportional;
		if(Math.abs(output) < 0.2){
			output = 0.2 * Math.signum(output);
		}
		if(Math.abs(output) > Robot.drivetrain.autonomousTwistSpeed) {
			output =   Robot.drivetrain.autonomousTwistSpeed*Math.signum(output);
		}
		
		RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, output, 0);
		//SmartDashboard.putNumber("twPID out", output);
    	
		executionCount++;
    	SmartDashboard.putNumber("Iterations", executionCount);
    	SmartDashboard.putNumber("Error", kError);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {

	return (Math.abs(kError) <  kTolerance);
		
	}

	// Called once after isFinished returns true
	protected void end() {
		System.out.println("RobotRotateCustomPID ended at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		System.out.println("RobotRotateCustomPID interrupted at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);

	}
}