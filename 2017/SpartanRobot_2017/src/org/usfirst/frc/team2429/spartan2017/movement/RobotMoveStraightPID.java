package org.usfirst.frc.team2429.spartan2017.movement;

import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.RobotMap;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RobotMoveStraightPID extends Command {
	private PIDController pid;
	private double heading;
	private double minimumSpeed = 0.05;
	private int executionCount;
	private double distance;

	

    public RobotMoveStraightPID(double distance) {
    	requires(Robot.drivetrain);
		pid = new PIDController(Robot.drivetrain.kPWheel, Robot.drivetrain.kIWheel, Robot.drivetrain.kDWheel, new PIDSource() {
			PIDSourceType m_sourceType = PIDSourceType.kDisplacement;

			@Override
			public double pidGet() {
				//return Robot.drivetrain.getDistanceToObstacle();
				return RobotMap.wheelEncoder.getDistance();
			}

			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {
				m_sourceType = pidSource;
			}

			@Override
			public PIDSourceType getPIDSourceType() {
				return m_sourceType;
			}
		}, new PIDOutput() {
			@Override
			public void pidWrite(double d) {
				//Robot.drivetrain.drive(d, d);
				double twist = Robot.drivetrain.kGyroProportional*(heading - RobotMap.gyro.getAngle());
				RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, - d + (minimumSpeed * Math.signum(-d)), twist, 0);
				
		    	executionCount++;
		    	SmartDashboard.putNumber("Iterations", executionCount);
		    	SmartDashboard.putNumber("Error", pid.getError());
		    	
				//SmartDashboard.putNumber("mvstrtPID out", d);
				//SmartDashboard.putNumber("twPID out",  - d + minimumSpeed * Math.signum(-d));
			}
		});
		pid.setOutputRange(-Robot.drivetrain.autonomousForwardSpeed, Robot.drivetrain.autonomousForwardSpeed);
		pid.setAbsoluteTolerance(1.0);
	
		// Use that boolean to see if we will use the distance passed in or the autonomous values ... may want to make this a switch
		//I'm sure there is a cleaner way to do this
		
		this.distance = distance;
		pid.setSetpoint(distance);

    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		// Get everything in a safe starting state.
    	System.out.println("RobotMoveStraightPID called at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    	Robot.drivetrain.reset();
    	heading=RobotMap.gyro.getAngle();
    	
		//I'm doing this twice in debugging to see if it will work for sure
    	pid.setSetpoint(distance);
		
    	//pid.setPID(Robot.drivetrain.kPWheel, Robot.drivetrain.kIWheel, Robot.drivetrain.kDWheel);
    	pid.reset();
		pid.enable();
    	executionCount = 0;
    	SmartDashboard.putString("Current Command", this.getClass().getSimpleName());
    	SmartDashboard.putNumber("Parameter", this.distance);
    }


    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
		return pid.onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	// Stop PID and the wheels
    	System.out.println("RobotMoveStraightPID ended at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    	pid.disable();
    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	// Stop PID and the wheels
    	System.out.println("RobotMoveStraightPID interrupted at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    	pid.disable();
    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
    }
}
