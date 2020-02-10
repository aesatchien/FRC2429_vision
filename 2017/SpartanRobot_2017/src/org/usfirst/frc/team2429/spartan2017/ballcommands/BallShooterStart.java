package org.usfirst.frc.team2429.spartan2017.ballcommands;

import org.usfirst.frc.team2429.spartan2017.Robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class BallShooterStart extends Command {

    public BallShooterStart() {
        
    	// Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("BallShooterStart called at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    	Robot.ballCollector.stop();
    	Robot.ballShooter.start();
    	Robot.ballGate.start();
    	Robot.ballAgitator.forward();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        //return !Robot.oi.buttonLB.get();
    	return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
