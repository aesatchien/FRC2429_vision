
package org.usfirst.frc2429.Spartan2018.autonomous;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc2429.Spartan2018.Robot;
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
public class TurnRobotPID extends Command {
	private PIDController pid;
	private int executionCount;
	private double minimum_turn_power = 0.6;
	private double angle;
	boolean bUseImaging=false;

	/**
	 *  Wrote this to either take an angle and false, or dummy angle and true 
	 *  If true, we fetch the saved value of the image processor variables 
	 *  Couldn't think of a cleaner way to make two constructors that did this
	 *  Has to do with the fact you can't change variables in a command group
	 */
	
	public TurnRobotPID(boolean bUseImaging) {
		this(Robot.imageProcessor.getCubecamAngle(),Robot.drivetrain.autonomousTwistSpeed,1.0);
		this.bUseImaging = true;
	}
	public TurnRobotPID(double angle) {
		this(angle,Robot.drivetrain.autonomousTwistSpeed,3.0);
		this.bUseImaging = false;
	}
	
	public TurnRobotPID(double angle, double maxSpeed, double tolerance) {
		requires(Robot.drivetrain);
		pid = new PIDController(Robot.drivetrain.kPtwistGyro, Robot.drivetrain.kItwistGyro, Robot.drivetrain.kDtwistGyro,
				new PIDSource() {
					PIDSourceType m_sourceType = PIDSourceType.kDisplacement;

					@Override
					public double pidGet() {
						return Robot.drivetrain.driveGyro.getAngle();
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
						if(Math.abs(d) < minimum_turn_power){
							d =minimum_turn_power * Math.signum(d);
						}
						
						Robot.drivetrain.drive(0, d);
						SmartDashboard.putNumber("twPID out", d);
					}
				});
		
		pid.setOutputRange(-maxSpeed, maxSpeed);	
		pid.setAbsoluteTolerance(tolerance);
		this.angle=angle;
		pid.setSetpoint(angle);

	}
	
@Deprecated
	public TurnRobotPID(double angle, boolean bUseImaging) {
		requires(Robot.drivetrain);
		pid = new PIDController(Robot.drivetrain.kPtwistGyro, Robot.drivetrain.kItwistGyro, Robot.drivetrain.kDtwistGyro,
				new PIDSource() {
					PIDSourceType m_sourceType = PIDSourceType.kDisplacement;

					@Override
					public double pidGet() {
						return Robot.drivetrain.driveGyro.getAngle();
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
						if(Math.abs(d) < minimum_turn_power){
							d =minimum_turn_power * Math.signum(d);
						}
						
						Robot.drivetrain.drive(0, d);
						SmartDashboard.putNumber("twPID out", d);
					}
				});
		pid.setOutputRange(-Robot.drivetrain.autonomousTwistSpeed, Robot.drivetrain.autonomousTwistSpeed);
		
		// Use that boolean to see if we will use the distance passed in or the
		// autonomous values ... may want to make this a switch
		// I'm sure there is a cleaner way to do this
		this.bUseImaging = bUseImaging;
		if (bUseImaging) {
			pid.setAbsoluteTolerance(1.0);
			this.angle=Robot.imageProcessor.getCubecamAngle();
			pid.setSetpoint(this.angle);
		} else {
			pid.setAbsoluteTolerance(3.0);
			this.angle=angle;
			pid.setSetpoint(angle);
			
		}

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		// Get everything in a safe starting state.
		System.out.println("TurnRobotPID called for: " + String.format("%.2f", angle)+ " degrees at "  + String.format("%.2f", Timer.getFPGATimestamp()- Robot.enabledTime)  + "s");
		Robot.drivetrain.driveGyro.reset();
		pid.setPID(Robot.drivetrain.kPtwistGyro, Robot.drivetrain.kItwistGyro, Robot.drivetrain.kDtwistGyro);
		
		//Have to reset these guys here if you are calling it from the dashboard - needs to update the goals
		if (bUseImaging) {
			//pid.setAbsoluteTolerance(1.0);
			this.angle=Robot.imageProcessor.getCubecamAngle();
			pid.setSetpoint(this.angle);
		}			else {
			pid.setSetpoint(this.angle);
			
		}
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
		System.out.println("TurnRobot ended at: " + String.format("%.2f", Timer.getFPGATimestamp()- Robot.enabledTime) + "s");
		pid.disable();
		Robot.drivetrain.drive(0, 0);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		// Stop PID and the wheels
		System.out.println("TurnRobot interrupted at: " + String.format("%.2f", Timer.getFPGATimestamp()- Robot.enabledTime) + "s");
		pid.disable();
		Robot.drivetrain.drive(0, 0);
	}
}
