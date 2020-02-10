package org.usfirst.frc2429.Spartan2018.autonomous;

import org.usfirst.frc2429.Spartan2018.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.TimedCommand;


public class TimedMove extends TimedCommand {
	
	static double velocity;
	/**
	 *  Timed move takes a velocity (positive or negative) and a timeout in s
	 */
    public TimedMove(double velocity, double timeout) {
        super(timeout);
        this.velocity = velocity;
        requires(Robot.drivetrain);

    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("\nEntering TimedMove at " + String.format("%.2f",Timer.getFPGATimestamp()) + "s");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drivetrain.drive(velocity, 0);
    }

    // Called once after timeout
    protected void end() {
    	System.out.println("Ending TimedMove at " + String.format("%.2f",Timer.getFPGATimestamp()) + "s");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("Iterrupting TimedMove at " + String.format("%.2f",Timer.getFPGATimestamp()) + "s");
    }
}
