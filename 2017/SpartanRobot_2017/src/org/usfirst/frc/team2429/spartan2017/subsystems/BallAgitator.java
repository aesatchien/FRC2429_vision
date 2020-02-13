// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc.team2429.spartan2017.subsystems;

import org.usfirst.frc.team2429.spartan2017.RobotMap;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *  TODO
 *  Need to determine which of the two approaches for a default command works best
 *  The command group version of oscillate or the separate thread version
 */
public class BallAgitator extends Subsystem {

    private final Victor ballAgitatorMotor = RobotMap.ballAgitatorMotor;
    public static final double oscillateTime = 0.5;
    public static final double agitatorSpeed = -0.4;
    public static Thread oscillateThread;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    
    /**
     *  TODO
     *  Need to determine which of the two approaches for a default command works best
     *  The command group version of oscillate or the separate thread version
     */
    public void initDefaultCommand() {

        // Set the default command for a subsystem here.
       // setDefaultCommand(new BallAgitatorForward());
    }
    
	/**
	 * Stop the ball agitator subsystem.
	 * Cleaning up the BallAgitator Subsystem - 02/04/2017 CJH
	 */
	public void stop() {
		ballAgitatorMotor.set(0.0);
	}
	
	/**
	 * Start the ball agitator subsystem moving forward.
	 * Cleaning up the BallAgitator Subsystem - 02/04/2017 CJH
	 */
	public void forward() {
		ballAgitatorMotor.set(agitatorSpeed);
	}
	
	/**
	 * Start the ball agitator subsystem moving in reverse.
	 * Cleaning up the BallAgitator Subsystem - 02/04/2017 CJH
	 */
	public void reverse() {
		ballAgitatorMotor.set(-1.0*agitatorSpeed);
	}
	
	/**
	 * Start the ball agitator subsystem oscillating.
	 * Not sure if this will work. I think delays are bad, especially long ones like this.
	 * Anyway, this is the loop.  Should loop until the stop command is called
	 * Cleaning up the BallAgitator Subsystem - 02/04/2017 CJH
	 */
	public void oscillate() {
		
		oscillateThread = new Thread(() -> {
		while (!Thread.interrupted()){
			forward();
			Timer.delay(oscillateTime);
			reverse();
			Timer.delay(oscillateTime);
		}
		});
		oscillateThread.setDaemon(true);
		oscillateThread.start();
	}
}
