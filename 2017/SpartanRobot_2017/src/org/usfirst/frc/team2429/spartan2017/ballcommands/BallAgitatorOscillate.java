package org.usfirst.frc.team2429.spartan2017.ballcommands;

import org.usfirst.frc.team2429.spartan2017.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class BallAgitatorOscillate extends Command {

    public BallAgitatorOscillate() {
    	 requires(Robot.ballAgitator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("BallAgitatorOscillate Called");
    	Robot.ballAgitator.oscillate();
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
    	System.out.println("BallAgitatorOscillate Ended");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("BallAgitatorOscillate Interrupted");
    }
}
