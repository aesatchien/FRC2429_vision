package org.usfirst.frc.team2429.spartan2017.subsystems;

import org.usfirst.frc.team2429.spartan2017.RobotMap;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RopeClimber extends Subsystem {

	private final Talon climberMotorL = RobotMap.climberMotorL;
	private final Talon climberMotorR = RobotMap.climberMotorR;
	
    public static final double climberSpeed = 1.0;
    
    // not used yet - we may need it if we want to lock out other systems when the climber is on 
    public static boolean bClimbing = false;


    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
	/**
	 * Reverse the ball collection subsystem.
	 * Cleaning up the BallCollector Subsystem - 02/04/2017 CJH
	 */
	public void start() {
		climberMotorL.set(1.0*climberSpeed);
		climberMotorR.set(-1.0*climberSpeed);
		bClimbing = true; 
		SmartDashboard.putBoolean("Climber", bClimbing);
	}
	
	public static boolean isClimbing() {
		return bClimbing;
	}

	/**
	 * Stop the ball collection subsystem.
	 * Cleaning up the BallCollector Subsystem - 02/04/2017 CJH
	 */
	public void stop() {
		climberMotorL.set(0.0);
		climberMotorR.set(0.0);
		bClimbing = false;
		SmartDashboard.putBoolean("Climber", bClimbing);
	}
	public void calibrate() {
		climberMotorL.set(1.0);
		climberMotorR.set(-1.0);
		Timer.delay(2);
		climberMotorL.set(-1.0);
		climberMotorR.set(1.0);
		Timer.delay(2);
		stop();
		
	}
	
}

