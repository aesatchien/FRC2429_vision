package org.usfirst.frc.team2429.spartan2017.movement;

import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.RobotMap;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RobotCenterOnGearPID extends Command {
	private PIDController pid;
	private double heading;
	private double minimumSpeed = 0.05;
	private boolean useLiveImage;
	private int executionCount;
	private NetworkTable gearTable;
	

    public RobotCenterOnGearPID(boolean useLiveImage) {
    	requires(Robot.drivetrain);
    	
		pid = new PIDController(Robot.drivetrain.kPWheel, Robot.drivetrain.kIWheel, Robot.drivetrain.kDWheel, new PIDSource() {
			PIDSourceType m_sourceType = PIDSourceType.kDisplacement;

			@Override
			public double pidGet() {
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

				double twist = Robot.drivetrain.kGyroProportional*(heading - RobotMap.gyro.getAngle());
				RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, - d + (minimumSpeed * Math.signum(-d)), twist, 0);
				
				//SmartDashboard.putNumber("mvstrtPID out", d);
				//SmartDashboard.putNumber("twPID out",  - d + minimumSpeed * Math.signum(-d));
		    	executionCount++;
		    	SmartDashboard.putNumber("Iterations", executionCount);
		    	SmartDashboard.putNumber("Error", pid.getError());
			}
		});
		pid.setOutputRange(-Robot.drivetrain.autonomousForwardSpeed, Robot.drivetrain.autonomousForwardSpeed);
		pid.setAbsoluteTolerance(1.0);
			
		//I'm sure there is a cleaner way to do this
		//I need a function that can either take a stored strafe pre-turn
		//or use the live image
		this.useLiveImage = useLiveImage;
		if (useLiveImage){
			pid.setSetpoint(Robot.imageProcessor.gearTargetStrafe);
		} else{
			pid.setSetpoint(Robot.imageProcessor.getStrafeCorrection());
		}
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		// Get everything in a safe starting state.
    	gearTable = NetworkTable.getTable("GearCam");
    	System.out.println("RobotCenterOnGearPID called at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    	Robot.drivetrain.reset();
    	heading=RobotMap.gyro.getAngle();
    	executionCount = 0;
    	SmartDashboard.putString("Current Command", this.getClass().getSimpleName());
    	SmartDashboard.putNumber("Parameter", Robot.imageProcessor.gearTargetStrafe);
    	
    	//I'm setting it twice just in case
		if (useLiveImage){
			// Added 3/21/2017 to speed things up: pid.setSetpoint( gearTable.getNumber("strafe", 0) );
			pid.setSetpoint(Robot.imageProcessor.gearTargetStrafe);
		} else{
			pid.setSetpoint(Robot.imageProcessor.getStrafeCorrection());
		}
    	pid.reset();
		pid.enable();
    }


    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
		return pid.onTarget();
		//Change to this eventually once we see what the bug was that showed up in Ventura
		//return (pid.onTarget() || isTimedOut() );
    }

    // Called once after isFinished returns true
    protected void end() {
    	// Stop PID and the wheels
    	System.out.println("RobotCenterOnGearPID ended at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    	pid.disable();
    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	// Stop PID and the wheels
    	System.out.println("RobotCenterOnGearPID interrupted at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    	pid.disable();
    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
    }
}
