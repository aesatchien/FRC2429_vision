package org.usfirst.frc.team2429.spartan2017.deprecated;

import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RobotCenterToGearTarget extends Command {

	public boolean isOperatorEnabled;
	double gyroAngle;
	double twist;
	double strafeTarget;
	double rotationTarget;
	double kProportionalStrafe = 0.05;
	double kProportionalTwist = 0.02;
	double yMotion;
	
	public RobotCenterToGearTarget() {

		isOperatorEnabled = true;
		// Use requires() here to declare subsystem dependencies
		requires(Robot.drivetrain);
		setTimeout(10);
	}

	// Called just before this Command runs the first time
	protected void initialize() {

		System.out.println("RobotCenterToGearTarget called at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");

		RobotMap.gyro.reset();
		gyroAngle = RobotMap.gyro.getAngle();

		strafeTarget = 5.0;
		rotationTarget = 5.0;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		yMotion = Math.min(Robot.drivetrain.autonomousGearDeliverySpeed, Robot.imageProcessor.gearTargetStrafe*kProportionalStrafe);
		twist = Math.min(Robot.imageProcessor.gearTargetRotation*kProportionalTwist, Robot.drivetrain.autonomousTwistSpeed);
		
		RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, -yMotion, twist, 0);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return (Math.abs(Robot.imageProcessor.gearTargetStrafe) < strafeTarget) && (Math.abs(Robot.imageProcessor.gearTargetRotation) < rotationTarget);  
	}

	// Called once after isFinished returns true
	protected void end() {

		if (isOperatorEnabled) {
			System.out.println(
					"RobotCenterToGearTarget ended at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		} else {
			System.out.println("RobotCenterToGsearTarget ended at: " + String.format("%.2f", Timer.getFPGATimestamp())
					+ "s   after running " + String.format("%.2f", timeSinceInitialized()) + "s");
		}
		RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		System.out.println(
				"RobotCenterToGearTarget interrupted at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);

	}
}
