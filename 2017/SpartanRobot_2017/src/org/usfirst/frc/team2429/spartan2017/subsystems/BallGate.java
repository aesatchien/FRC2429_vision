package org.usfirst.frc.team2429.spartan2017.subsystems;

import org.usfirst.frc.team2429.spartan2017.RobotMap;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class BallGate extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	
	//This was a servo.  Now it's broken.  - 02/09/2017
	private final Victor ballGateMotor = RobotMap.ballGateMotor;
    public static final double gateSpeed = -1.0;
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
	
	/**
	 * Start the ball gate motor and wait for it to finish.
	 * Cleaning up the BallGate Subsystem - 02/04/2017 CJH
	 */
	public void stop() {
		ballGateMotor.set(0.0);
	}
	
	/**
	 * Close the ball gate servo and wait for it to finish.
	 * Cleaning up the BallGate Subsystem - 02/04/2017 CJH
	 */
	public void start() {
		ballGateMotor.set(gateSpeed);

	}
}
