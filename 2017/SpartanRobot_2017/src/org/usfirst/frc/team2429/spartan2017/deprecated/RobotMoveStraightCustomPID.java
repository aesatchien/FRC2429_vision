package org.usfirst.frc.team2429.spartan2017.deprecated;

import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RobotMoveStraightCustomPID extends Command {

	public boolean bUseImaging;
	
	
	private double setpoint;
	private double kError;
	private double kTolerance = 0.5;
	private double speed;
	private double heading;
	
	
	//private double startingAngle;

	public RobotMoveStraightCustomPID(double distance, boolean bUseImaging) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.drivetrain);
				
		//We had two ways of doing this - reset the gyro or take the current value
		//For some reason this does not work the second time through - breaks after 1st use
		//so I'm sticking with the resetting of the gyro for now
		//startingAngle = RobotMap.gyro.getAngle();
		this.bUseImaging = bUseImaging;
		this.setpoint = distance;

	}

	

	// Called just before this Command runs the first time
	protected void initialize() {
		
		if (bUseImaging) {
			setpoint=Robot.imageProcessor.autonomousGearStrafe;
		} else {
			// was set in constructor
		}
		
		System.out.println("RobotMoveStraightCustomPID called at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		RobotMap.gyro.reset();
    	heading=RobotMap.gyro.getAngle();
		//kTolerance = 0.2 + 0.06*Math.abs(setpoint);
    	RobotMap.wheelEncoder.reset();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double output;
		
		kError = setpoint - RobotMap.wheelEncoder.getDistance();
		double twist = Robot.drivetrain.kGyroProportional*(heading - RobotMap.gyro.getAngle());
		output = kError * Robot.drivetrain.kForwardProportional;
		if(Math.abs(output) < 0.2){
			output = 0.2 * Math.signum(output);
		}
		if(Math.abs(output) > Robot.drivetrain.autonomousForwardSpeed) {
			output = Robot.drivetrain.autonomousForwardSpeed*Math.signum(output);
		}
		
		if((Math.abs(setpoint) > 10) && (kError < (0.1*setpoint))){
			output = -0.0;
		}
		
		RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, -output, twist, 0);
		SmartDashboard.putNumber("twPID out", output);

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {

	
	return (Math.abs(kError) <  kTolerance);
		
	}

	// Called once after isFinished returns true
	protected void end() {
		System.out.println("RobotMoveStraightCustomPID ended at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		System.out.println("RobotMoveStraightCustomPID interrupted at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);

	}
}