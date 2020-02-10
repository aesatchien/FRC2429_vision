package org.usfirst.frc.team2429.spartan2017.deprecated;

import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RobotStrafeDiagonal extends Command {

	public boolean isOperatorEnabled;
	double x = 0;
	double y = 0;
	double gyroAngle;
	double twist;
	
	public RobotStrafeDiagonal(double timeout, double x, double y) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.drivetrain);
		
		this.x = x;
		this.y = y;
		
		isOperatorEnabled = false;

		setTimeout(timeout);

	}
	
    public RobotStrafeDiagonal() {
        // Use requires() here to declare subsystem dependencies
    	isOperatorEnabled = true;
        requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("RobotStrafeDiagonal called at: " + String.format("%.2f",Timer.getFPGATimestamp())+"s");
    	RobotMap.gyro.reset();
    	gyroAngle = RobotMap.gyro.getAngle();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	twist = (gyroAngle - RobotMap.gyro.getAngle())*Robot.drivetrain.kGyroProportional;
    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(Robot.drivetrain.dpadStrafeSpeed*x, Robot.drivetrain.dpadStrafeSpeed*y, twist, 0);
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if (isOperatorEnabled) {
			return !Robot.oi.buttonPovRight.get();
		} else {
			return isTimedOut();
		}
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("RobotStrafeDiagonal ended at: " + String.format("%.2f",Timer.getFPGATimestamp())+"s");
    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("RobotStrafeDiagonal interrupeted at: " + String.format("%.2f",Timer.getFPGATimestamp())+"s");
    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
    }
}
