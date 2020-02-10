package org.usfirst.frc.team2429.spartan2017.commands;

import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ClimberStart extends Command {

	private double testPower;

	public ClimberStart() {
		requires(Robot.ropeClimber);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		System.out.println("ClimberStart called at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		Robot.ropeClimber.start();
		Robot.ballShooter.stop();
		Robot.ballAgitator.stop();
		Robot.ballCollector.stop();
		Robot.ballGate.stop();

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	protected void end() {
		System.out.println("ClimberStart ended at: " + String.format("%.2f", Timer.getFPGATimestamp())
				+ "s   after running " + String.format("%.2f", timeSinceInitialized()) + "s");

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
