package org.usfirst.frc.team2429.spartan2017.subsystems;

import org.usfirst.frc.team2429.spartan2017.RobotMap;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class BallShooter extends Subsystem {

	public final CANTalon ballShooterMotor = RobotMap.ballShooterMotor;
	public static final double shooterSpeed = -1.0;
	private static final double shooterRPM = -2750;
	public static boolean bShooting = false;

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	/**
	 * Start the ball shooter subsystem. TODO: change this to be a set speed
	 * with shot distances Cleaning up the BallShooter Subsystem - 02/04/2017
	 * CJH
	 */
	public void start() {

		//System.out.println("Shooter F: " + ballShooterMotor.getF());
		//System.out.println("Shooter P: " + ballShooterMotor.getP());
		//System.out.println("Shooter D: " + ballShooterMotor.getD());
		//System.out.println("Shooter I: " + ballShooterMotor.getI());
		//System.out.println("Shooter Mode: " + ballShooterMotor.getControlMode());

		//ballShooterMotor.changeControlMode(TalonControlMode.PercentVbus);
		ballShooterMotor.changeControlMode(TalonControlMode.Speed);
		ballShooterMotor.configPeakOutputVoltage(0.0f, -12.0f); // Only Negative
																// Throttle

		ballShooterMotor.setProfile(0);
		// Edit the following values: http://roborio-2429-frc.local/#Home
		// ballShooterMotor.setF(0.05);
		// ballShooterMotor.setP(0.1);
		// ballShooterMotor.setI(0.0);
		// ballShooterMotor.setD(0.0);

		ballShooterMotor.setSetpoint(shooterRPM);
		bShooting = true; 
		SmartDashboard.putBoolean("Shooter", bShooting);
		//ballShooterMotor.set(shooterSpeed);

	}

	/**
	 * Stop the ball shooter subsystem. Cleaning up the BallShooter Subsystem -
	 * 02/04/2017 CJH
	 */
	public void stop() {
		ballShooterMotor.set(0.0);
		bShooting = false; 
		SmartDashboard.putBoolean("Shooter", bShooting);
	}
}
