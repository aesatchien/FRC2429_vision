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
public class RobotTwistPID extends Command {
	private PIDController pid;
	private int executionCount;

	/**
	 *  Wrote this to either take an angle and false, or dummy angle and true 
	 *  If true, we fetch the saved value of the image processor variables 
	 *  Couldn't think of a cleaner way to make two constructors that did this
	 *  Has to do with the fact you can't change variables in a command group
	 */
	
	public RobotTwistPID(double angle, boolean bUseImaging) {
		requires(Robot.drivetrain);
		pid = new PIDController(Robot.drivetrain.kPGyro, Robot.drivetrain.kIGyro, Robot.drivetrain.kDGyro,
				new PIDSource() {
					PIDSourceType m_sourceType = PIDSourceType.kDisplacement;

					@Override
					public double pidGet() {
						// return Robot.drivetrain.getDistanceToObstacle();
						return RobotMap.gyro.getAngle();
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
						// Robot.drivetrain.drive(d, d);
						if(Math.abs(d) < 0.2){
							d = 0.2 * Math.signum(d);
						}
						
						RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, d, 0);
						SmartDashboard.putNumber("twPID out", d);
					}
				});
		pid.setOutputRange(-Robot.drivetrain.autonomousTwistSpeed, Robot.drivetrain.autonomousTwistSpeed);
		pid.setAbsoluteTolerance(1.0);
		
		
	
		// Use that boolean to see if we will use the distance passed in or the
		// autonomous values ... may want to make this a switch
		// I'm sure there is a cleaner way to do this
		if (bUseImaging) {
			pid.setSetpoint(Robot.imageProcessor.autonomousGearRotation);
		} else {
			pid.setSetpoint(angle);
		}

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		// Get everything in a safe starting state.
		System.out.println("RobotTwistPID called at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		RobotMap.gyro.reset();
		pid.setPID(Robot.drivetrain.kPGyro, Robot.drivetrain.kIGyro, Robot.drivetrain.kDGyro);
		pid.reset();
		pid.enable();
    	executionCount = 0;
    	SmartDashboard.putString("Current Command", this.getClass().getSimpleName());
    	SmartDashboard.putNumber("Parameter", pid.getSetpoint());
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return pid.onTarget();
	}

	// Called once after isFinished returns true
	protected void end() {
		// Stop PID and the wheels
		System.out.println("RobotTwistPID ended at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		pid.disable();
		RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		// Stop PID and the wheels
		System.out.println("RobotTwistPID interrupted at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		pid.disable();
		RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
	}
}
