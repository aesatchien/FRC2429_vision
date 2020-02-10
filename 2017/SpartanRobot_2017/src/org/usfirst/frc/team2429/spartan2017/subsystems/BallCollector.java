package org.usfirst.frc.team2429.spartan2017.subsystems;

import org.usfirst.frc.team2429.spartan2017.RobotMap;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class BallCollector extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	private final Victor ballCollectorMotor = RobotMap.ballCollectorMotor;
	//private final Talon ballCollectorMotorR = RobotMap.ballCollectorMotorR;
	
    public static final double collectorSpeed = 0.65;
    //public static final double collectorSpeedR = 1.0;
    public static boolean bCollecting = false;
    

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
	
	/**
	 * Start the ball collection subsystem.
	 * Cleaning up the BallCollector Subsystem - 02/04/2017 CJH
	 */
	public void start() {
		ballCollectorMotor.set(collectorSpeed);
		//ballCollectorMotorR.set(collectorSpeedR);
		bCollecting = true;
		SmartDashboard.putBoolean("Collector", bCollecting);
	}
	
	/**
	 * Reverse the ball collection subsystem.
	 * Cleaning up the BallCollector Subsystem - 02/04/2017 CJH
	 */
	public void reverse() {
		ballCollectorMotor.set(-1.0*collectorSpeed);
		bCollecting = true;
		SmartDashboard.putBoolean("Collector", bCollecting);
		//ballCollectorMotorR.set(-1.0*collectorSpeedR);
	}
	
	/**
	 * Stop the ball collection subsystem.
	 * Cleaning up the BallCollector Subsystem - 02/04/2017 CJH
	 */
	public void stop() {
		ballCollectorMotor.set(0.0);
		bCollecting = false;
		SmartDashboard.putBoolean("Collector", bCollecting);
		//ballCollectorMotorR.set(0.0);
	}
}
